import math
from dataclasses import dataclass


@dataclass
class OccupancyGridHelper:
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: list

    @classmethod
    def from_msg(cls, msg):
        q = msg.info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return cls(
            resolution=msg.info.resolution,
            width=msg.info.width,
            height=msg.info.height,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            origin_yaw=origin_yaw,
            data=list(msg.data),
        )

    def world_to_map(self, x: float, y: float):
        dx = x - self.origin_x
        dy = y - self.origin_y
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        gx = c * dx + s * dy
        gy = -s * dx + c * dy
        mx = int(math.floor(gx / self.resolution))
        my = int(math.floor(gy / self.resolution))
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return None
        return mx, my

    def map_to_world(self, mx: int, my: int):
        gx = (mx + 0.5) * self.resolution
        gy = (my + 0.5) * self.resolution
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        wx = self.origin_x + c * gx - s * gy
        wy = self.origin_y + s * gx + c * gy
        return wx, wy

    def is_occupied(self, x: float, y: float, threshold: int = 50, treat_unknown_as_obstacle: bool = False) -> bool:
        idx = self.world_to_map(x, y)
        if idx is None:
            return treat_unknown_as_obstacle
        mx, my = idx
        value = self.data[my * self.width + mx]
        if value < 0:
            return treat_unknown_as_obstacle
        return value >= threshold


def create_obstacle_grid_from_occupancy(msg, occupied_threshold: int = 50, treat_unknown_as_obstacle: bool = False):
    """Return pts and signed-distance-like values from an OccupancyGrid.

    For simplicity, free cells are +1.0, occupied/unknown are -1.0.
    """
    helper = OccupancyGridHelper.from_msg(msg)
    pts = []
    cells = []

    for my in range(helper.height):
        for mx in range(helper.width):
            wx, wy = helper.map_to_world(mx, my)
            value = helper.data[my * helper.width + mx]
            if value >= occupied_threshold or (value < 0 and treat_unknown_as_obstacle):
                cells.append(-1.0)
            else:
                cells.append(1.0)
            pts.append((wx, wy))

    return helper, pts, cells

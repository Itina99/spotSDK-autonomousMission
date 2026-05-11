import math
import heapq
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


@dataclass
class ObstacleDistanceField:
    helper: OccupancyGridHelper
    signed_cells: list
    occupied_mask: list
    unknown_mask: list


def create_obstacle_grid_from_occupancy(msg, occupied_threshold: int = 50, treat_unknown_as_obstacle: bool = False):
    """Return sampled world points and signed obstacle-distance values.

    Distances are in meters and follow SDK-style semantics:
    - negative -> inside obstacle space
    - zero     -> obstacle boundary
    - positive -> free space
    """
    helper = OccupancyGridHelper.from_msg(msg)
    cells, _, _ = compute_signed_obstacle_distance(
        helper,
        occupied_threshold=occupied_threshold,
        treat_unknown_as_obstacle=treat_unknown_as_obstacle,
    )
    pts = []

    for my in range(helper.height):
        for mx in range(helper.width):
            wx, wy = helper.map_to_world(mx, my)
            pts.append((wx, wy))

    return helper, pts, cells


def build_obstacle_distance_field(msg, occupied_threshold: int = 50, treat_unknown_as_obstacle: bool = False):
    helper = OccupancyGridHelper.from_msg(msg)
    signed_cells, occupied_mask, unknown_mask = compute_signed_obstacle_distance(
        helper,
        occupied_threshold=occupied_threshold,
        treat_unknown_as_obstacle=treat_unknown_as_obstacle,
    )
    return ObstacleDistanceField(
        helper=helper,
        signed_cells=signed_cells,
        occupied_mask=occupied_mask,
        unknown_mask=unknown_mask,
    )


def build_occupied_mask(helper: OccupancyGridHelper, occupied_threshold: int = 50, treat_unknown_as_obstacle: bool = False):
    occupied = []
    unknown = []
    for value in helper.data:
        is_unknown = value < 0
        unknown.append(is_unknown)
        if is_unknown:
            occupied.append(bool(treat_unknown_as_obstacle))
        else:
            occupied.append(value >= occupied_threshold)
    return occupied, unknown


def _multi_source_distance(width: int, height: int, resolution: float, seed_mask):
    total = width * height
    inf = float('inf')
    dist = [inf] * total
    heap = []

    for idx, is_seed in enumerate(seed_mask):
        if is_seed:
            dist[idx] = 0.0
            heapq.heappush(heap, (0.0, idx))

    if not heap:
        return dist

    neighbors = [
        (-1, 0, resolution),
        (1, 0, resolution),
        (0, -1, resolution),
        (0, 1, resolution),
        (-1, -1, resolution * math.sqrt(2.0)),
        (-1, 1, resolution * math.sqrt(2.0)),
        (1, -1, resolution * math.sqrt(2.0)),
        (1, 1, resolution * math.sqrt(2.0)),
    ]

    while heap:
        current, idx = heapq.heappop(heap)
        if current > dist[idx]:
            continue

        x = idx % width
        y = idx // width
        for dx, dy, cost in neighbors:
            nx = x + dx
            ny = y + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            nidx = ny * width + nx
            alt = current + cost
            if alt < dist[nidx]:
                dist[nidx] = alt
                heapq.heappush(heap, (alt, nidx))

    return dist


def compute_signed_obstacle_distance(
    helper: OccupancyGridHelper,
    occupied_threshold: int = 50,
    treat_unknown_as_obstacle: bool = False,
):
    occupied_mask, unknown_mask = build_occupied_mask(
        helper,
        occupied_threshold=occupied_threshold,
        treat_unknown_as_obstacle=treat_unknown_as_obstacle,
    )
    free_mask = [not value for value in occupied_mask]
    fallback = max(helper.width, helper.height) * helper.resolution

    if any(occupied_mask):
        dist_to_occ = _multi_source_distance(helper.width, helper.height, helper.resolution, occupied_mask)
    else:
        dist_to_occ = [fallback] * (helper.width * helper.height)

    if any(free_mask):
        dist_to_free = _multi_source_distance(helper.width, helper.height, helper.resolution, free_mask)
    else:
        dist_to_free = [fallback] * (helper.width * helper.height)

    signed_cells = []
    for idx, is_occ in enumerate(occupied_mask):
        if is_occ:
            signed_cells.append(-dist_to_free[idx])
        else:
            signed_cells.append(dist_to_occ[idx])

    return signed_cells, occupied_mask, unknown_mask


def distance_at_world(
    helper: OccupancyGridHelper,
    signed_cells,
    x: float,
    y: float,
    out_of_map_value=float('inf'),
):
    idx = helper.world_to_map(x, y)
    if idx is None:
        return out_of_map_value
    mx, my = idx
    return signed_cells[my * helper.width + mx]


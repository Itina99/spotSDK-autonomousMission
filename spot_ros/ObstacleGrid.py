import math
from dataclasses import dataclass

import numpy as np


@dataclass
class GridSpec:
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: list

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


@dataclass
class ObstacleGrid:
    """SDK-like obstacle distance grid helper.

    Signed distance semantics:
    - negative -> inside obstacle space
    - zero     -> obstacle boundary
    - positive -> free space
    """

    spec: GridSpec
    signed_cells: list
    occupied_mask: list
    unknown_mask: list

    @classmethod
    def from_occupancy_grid_msg(
        cls,
        msg,
        occupied_threshold: int = 50,
        treat_unknown_as_obstacle: bool = False,
    ):
        spec = _spec_from_msg(msg)
        signed_cells, occupied_mask, unknown_mask = _compute_signed_obstacle_distance(
            spec,
            occupied_threshold=occupied_threshold,
            treat_unknown_as_obstacle=treat_unknown_as_obstacle,
        )
        return cls(
            spec=spec,
            signed_cells=signed_cells,
            occupied_mask=occupied_mask,
            unknown_mask=unknown_mask,
        )

    @classmethod
    def from_grid_data(
        cls,
        resolution: float,
        width: int,
        height: int,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
        data,
        occupied_threshold: int = 50,
        treat_unknown_as_obstacle: bool = False,
    ):
        spec = GridSpec(
            resolution=resolution,
            width=width,
            height=height,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
            data=list(data),
        )
        signed_cells, occupied_mask, unknown_mask = _compute_signed_obstacle_distance(
            spec,
            occupied_threshold=occupied_threshold,
            treat_unknown_as_obstacle=treat_unknown_as_obstacle,
        )
        return cls(
            spec=spec,
            signed_cells=signed_cells,
            occupied_mask=occupied_mask,
            unknown_mask=unknown_mask,
        )

    def distance_at(self, x: float, y: float, out_of_map_value=float('inf')):
        idx = self.spec.world_to_map(x, y)
        if idx is None:
            return out_of_map_value
        mx, my = idx
        return self.signed_cells[my * self.spec.width + mx]

    def to_points_and_cells(self):
        points = np.zeros((self.spec.width * self.spec.height, 2), dtype=np.float32)
        cells = np.asarray(self.signed_cells, dtype=np.float32)
        k = 0
        for my in range(self.spec.height):
            for mx in range(self.spec.width):
                wx, wy = self.spec.map_to_world(mx, my)
                points[k, 0] = wx
                points[k, 1] = wy
                k += 1
        return points, cells


def _spec_from_msg(msg) -> GridSpec:
    q = msg.info.origin.orientation
    origin_yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return GridSpec(
        resolution=msg.info.resolution,
        width=msg.info.width,
        height=msg.info.height,
        origin_x=msg.info.origin.position.x,
        origin_y=msg.info.origin.position.y,
        origin_yaw=origin_yaw,
        data=list(msg.data),
    )


def _build_occupied_mask(
    spec: GridSpec,
    occupied_threshold: int = 50,
    treat_unknown_as_obstacle: bool = False,
):
    occupied = []
    unknown = []
    for value in spec.data:
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
            heap.append((0.0, idx))

    if not heap:
        return dist

    import heapq

    heapq.heapify(heap)
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


def _compute_signed_obstacle_distance(
    spec: GridSpec,
    occupied_threshold: int = 50,
    treat_unknown_as_obstacle: bool = False,
):
    occupied_mask, unknown_mask = _build_occupied_mask(
        spec,
        occupied_threshold=occupied_threshold,
        treat_unknown_as_obstacle=treat_unknown_as_obstacle,
    )
    free_mask = [not value for value in occupied_mask]
    fallback = max(spec.width, spec.height) * spec.resolution

    if any(occupied_mask):
        dist_to_occ = _multi_source_distance(spec.width, spec.height, spec.resolution, occupied_mask)
    else:
        dist_to_occ = [fallback] * (spec.width * spec.height)

    if any(free_mask):
        dist_to_free = _multi_source_distance(spec.width, spec.height, spec.resolution, free_mask)
    else:
        dist_to_free = [fallback] * (spec.width * spec.height)

    signed_cells = []
    for idx, is_occ in enumerate(occupied_mask):
        if is_occ:
            signed_cells.append(-dist_to_free[idx])
        else:
            signed_cells.append(dist_to_occ[idx])

    return signed_cells, occupied_mask, unknown_mask


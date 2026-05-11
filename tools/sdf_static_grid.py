#!/usr/bin/env python3
"""
Generate a 2D occupancy grid from a Gazebo SDF world and visualize it.

This is the main tool for parsing SDF files and building occupancy grids.
Supports multiple output formats (matplotlib, ASCII, CSV).

Quick usage:
  python3 sdf_static_grid.py --plot              # Show matplotlib visualization
  python3 sdf_static_grid.py --save-image out.png # Save as PNG
  python3 sdf_static_grid.py --ascii             # Print ASCII grid
  python3 sdf_static_grid.py --save-csv data.csv # Export as CSV

For a simpler, user-friendly visualization:
  python3 visualize_grid.py
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Iterable, List, Tuple


@dataclass
class BoxObstacle:
    name: str
    x: float
    y: float
    sx: float
    sy: float
    yaw: float

    def aabb(self) -> Tuple[float, float, float, float]:
        """Return axis-aligned bounds (min_x, max_x, min_y, max_y)."""
        hx = self.sx * 0.5
        hy = self.sy * 0.5
        corners = [
            (hx, hy),
            (hx, -hy),
            (-hx, hy),
            (-hx, -hy),
        ]
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        xs = []
        ys = []
        for cx, cy in corners:
            rx = c * cx - s * cy + self.x
            ry = s * cx + c * cy + self.y
            xs.append(rx)
            ys.append(ry)
        return min(xs), max(xs), min(ys), max(ys)


@dataclass
class GridSpec:
    size: int
    grid_range: float

    @property
    def resolution(self) -> float:
        return (2.0 * self.grid_range) / float(self.size)

    @property
    def origin_x(self) -> float:
        return -self.grid_range

    @property
    def origin_y(self) -> float:
        return -self.grid_range


@dataclass
class OccupancyGrid:
    spec: GridSpec
    data: List[List[int]]

    def occupied_count(self) -> int:
        return sum(1 for row in self.data for v in row if v > 0)

    def to_csv(self, path: str) -> None:
        with open(path, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerows(self.data)


def _parse_pose(text: str) -> Tuple[float, float, float, float, float, float]:
    values = [float(v) for v in text.strip().split()]
    while len(values) < 6:
        values.append(0.0)
    return tuple(values[:6])  # x, y, z, roll, pitch, yaw


def _find_box_size(model: ET.Element) -> Tuple[float, float, float] | None:
    size_elem = model.find(".//geometry/box/size")
    if size_elem is None or size_elem.text is None:
        return None
    sx, sy, sz = (float(v) for v in size_elem.text.strip().split())
    return sx, sy, sz


def parse_sdf_obstacles(sdf_path: str, ignore_names: Iterable[str]) -> List[BoxObstacle]:
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    ignore_set = {name.strip() for name in ignore_names}
    obstacles: List[BoxObstacle] = []

    for model in root.findall(".//model"):
        name = model.attrib.get("name", "")
        if name in ignore_set:
            continue
        size = _find_box_size(model)
        if size is None:
            continue
        pose_elem = model.find("pose")
        if pose_elem is None or pose_elem.text is None:
            pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            pose = _parse_pose(pose_elem.text)
        x, y, _z, _r, _p, yaw = pose
        sx, sy, _sz = size
        obstacles.append(BoxObstacle(name=name, x=x, y=y, sx=sx, sy=sy, yaw=yaw))

    return obstacles


def _world_to_grid(spec: GridSpec, x: float, y: float) -> Tuple[int, int]:
    gx = int(math.floor((x - spec.origin_x) / spec.resolution))
    gy = int(math.floor((y - spec.origin_y) / spec.resolution))
    return gx, gy


def build_occupancy_grid(spec: GridSpec, obstacles: List[BoxObstacle]) -> OccupancyGrid:
    grid = [[0 for _ in range(spec.size)] for _ in range(spec.size)]

    for obs in obstacles:
        min_x, max_x, min_y, max_y = obs.aabb()
        gx0, gy0 = _world_to_grid(spec, min_x, min_y)
        gx1, gy1 = _world_to_grid(spec, max_x, max_y)

        x_start = max(0, min(gx0, gx1))
        x_end = min(spec.size - 1, max(gx0, gx1))
        y_start = max(0, min(gy0, gy1))
        y_end = min(spec.size - 1, max(gy0, gy1))

        for gy in range(y_start, y_end + 1):
            row = grid[gy]
            for gx in range(x_start, x_end + 1):
                row[gx] = 100

    return OccupancyGrid(spec=spec, data=grid)


def render_ascii(grid: OccupancyGrid, scale: int) -> str:
    step = max(1, scale)
    rows = []
    for y in range(0, grid.spec.size, step):
        row = grid.data[grid.spec.size - 1 - y]
        chars = ["#" if row[x] > 0 else "." for x in range(0, grid.spec.size, step)]
        rows.append("".join(chars))
    return "\n".join(rows)


def maybe_plot(grid: OccupancyGrid, save_path: str | None) -> None:
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        raise RuntimeError("matplotlib is required for plotting; install it or use --ascii")

    extent = [
        grid.spec.origin_x,
        grid.spec.origin_x + 2.0 * grid.spec.grid_range,
        grid.spec.origin_y,
        grid.spec.origin_y + 2.0 * grid.spec.grid_range,
    ]
    data = list(reversed(grid.data))
    plt.figure(figsize=(5, 5))
    plt.imshow(
        data,
        cmap="gray_r",
        origin="lower",
        extent=extent,
        interpolation="nearest",
    )
    plt.gca().set_aspect("equal", adjustable="box")
    plt.title("Occupancy Grid")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.colorbar(label="occupancy")
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
    else:
        plt.show()


def _default_sdf_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "spot", "test.sdf"))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build a static occupancy grid from an SDF world.")
    parser.add_argument("--sdf", default=_default_sdf_path(), help="Path to SDF world file")
    parser.add_argument("--grid-size", type=int, default=60, help="Grid size in cells (square)")
    parser.add_argument("--grid-range", type=float, default=3.0, help="World range (+/- meters)")
    parser.add_argument("--ignore", default="ground_plane,spot", help="Comma-separated model names to ignore")
    parser.add_argument("--ascii", action="store_true", help="Print ASCII grid to stdout")
    parser.add_argument("--ascii-scale", type=int, default=1, help="Downsample factor for ASCII output")
    parser.add_argument("--plot", action="store_true", help="Show matplotlib visualization (default: True if no other output)")
    parser.add_argument("--no-plot", action="store_true", help="Disable matplotlib visualization")
    parser.add_argument("--save-image", default="", help="Save matplotlib visualization to image")
    parser.add_argument("--save-csv", default="", help="Save grid to CSV file")
    parser.add_argument("--self-test", action="store_true", help="Run a simple parse/grid sanity check")
    return parser


def run_self_test(grid: OccupancyGrid, obstacles: List[BoxObstacle]) -> int:
    if not obstacles:
        print("Self-test failed: no obstacles parsed.")
        return 1
    if grid.occupied_count() == 0:
        print("Self-test failed: grid has no occupied cells.")
        return 1
    print("Self-test ok.")
    return 0


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    ignore_names = [name for name in args.ignore.split(",") if name.strip()]
    obstacles = parse_sdf_obstacles(args.sdf, ignore_names)

    spec = GridSpec(size=args.grid_size, grid_range=args.grid_range)
    grid = build_occupancy_grid(spec, obstacles)

    print(f"Loaded obstacles: {len(obstacles)}")
    print(f"Grid size: {spec.size}x{spec.size}")
    print(f"Resolution: {spec.resolution:.3f} m/cell")
    print(f"Occupied cells: {grid.occupied_count()}")

    if args.save_csv:
        grid.to_csv(args.save_csv)
        print(f"Saved CSV: {args.save_csv}")

    if args.ascii:
        print(render_ascii(grid, args.ascii_scale))

    # Default: show plot unless explicitly disabled
    show_plot = (args.plot or not args.no_plot) and not args.ascii and not args.self_test
    if show_plot or args.save_image:
        save_path = args.save_image or None
        maybe_plot(grid, save_path)

    if args.self_test:
        return run_self_test(grid, obstacles)

    return 0


if __name__ == "__main__":
    sys.exit(main())


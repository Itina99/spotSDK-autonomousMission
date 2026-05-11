#!/usr/bin/env python3
"""Load static occupancy grid from SDF file."""

from dataclasses import dataclass
import sys
import os

# Import tools module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

from sdf_static_grid import parse_sdf_obstacles, GridSpec, build_occupancy_grid
from visualize_grid import calculate_grid_range


@dataclass
class StaticGridCache:
    """Cached static grid data (flattened)."""
    grid_data: list
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


def load_static_grid(sdf_path: str) -> StaticGridCache:
    """Load SDF and build occupancy grid. Called once at startup."""
    ignore = ["ground_plane", "spot"]
    obstacles = parse_sdf_obstacles(sdf_path, ignore)
    grid_range = calculate_grid_range(obstacles)
    spec = GridSpec(size=60, grid_range=grid_range)
    grid = build_occupancy_grid(spec, obstacles)
    flat_data = [v for row in grid.data for v in row]

    return StaticGridCache(
        grid_data=flat_data,
        width=spec.size,
        height=spec.size,
        resolution=spec.resolution,
        origin_x=spec.origin_x,
        origin_y=spec.origin_y,
    )


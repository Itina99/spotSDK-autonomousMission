#!/usr/bin/env python3
"""
Quick visualization of the static occupancy grid from SDF.
This is a simplified wrapper around sdf_static_grid.py for end-user visualization.
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sdf_static_grid import (
    build_occupancy_grid,
    GridSpec,
    parse_sdf_obstacles,
)


def calculate_grid_range(obstacles):
    """Calculate optimal grid range to fit all obstacles with 20% margin."""
    if not obstacles:
        return 3.0

    xs = [obs.x for obs in obstacles]
    ys = [obs.y for obs in obstacles]

    max_x = max(abs(min(xs)), abs(max(xs)))
    max_y = max(abs(min(ys)), abs(max(ys)))
    needed_range = max(max_x, max_y)

    # Add 20% margin and round up to nearest 0.5
    margin = needed_range * 0.2
    required = needed_range + margin
    rounded = ((required // 0.5) + 1) * 0.5

    return rounded


def simple_plot(grid, obstacles, title="Occupancy Grid from SDF"):
    """Simple matplotlib visualization with details."""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("Error: matplotlib and numpy are required. Install with: pip install matplotlib numpy")
        return False

    # Prepare grid data
    data = list(reversed(grid.data))

    # Create figure with better settings
    fig, ax = plt.subplots(figsize=(10, 10), dpi=100)

    # Plot occupancy grid
    extent = [
        grid.spec.origin_x,
        grid.spec.origin_x + 2.0 * grid.spec.grid_range,
        grid.spec.origin_y,
        grid.spec.origin_y + 2.0 * grid.spec.grid_range,
    ]

    im = ax.imshow(
        data,
        cmap="RdYlGn_r",  # Red for occupied, Yellow for uncertain, Green for free
        origin="lower",
        extent=extent,
        interpolation="nearest",
        vmin=0,
        vmax=100,
    )

    # Formatting
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.set_xlabel("X [m]", fontsize=12)
    ax.set_ylabel("Y [m]", fontsize=12)
    ax.grid(True, alpha=0.3, linestyle="--")

    # Colorbar
    cbar = plt.colorbar(im, ax=ax, label="Occupancy (%)")

    # Statistics
    occupied = grid.occupied_count()
    total = grid.spec.size * grid.spec.size
    free = total - occupied

    stats_text = f"Occupied: {occupied} cells ({100*occupied/total:.1f}%)\nFree: {free} cells ({100*free/total:.1f}%)\nResolution: {grid.spec.resolution:.3f} m/cell"
    ax.text(
        0.02, 0.98, stats_text,
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
    )

    plt.tight_layout()
    plt.show()
    return True


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Visualize the static occupancy grid from Gazebo SDF"
    )
    parser.add_argument(
        "--sdf",
        default=os.path.join(os.path.dirname(__file__), "..", "spot", "test.sdf"),
        help="Path to SDF world file",
    )
    parser.add_argument(
        "--grid-size",
        type=int,
        default=60,
        help="Grid size in cells (square grid)",
    )
    parser.add_argument(
        "--grid-range",
        type=float,
        default=None,
        help="World range in ± meters from origin (auto-calculated if not set)",
    )
    parser.add_argument(
        "--ignore",
        default="ground_plane,spot",
        help="Comma-separated model names to ignore",
    )

    args = parser.parse_args()

    print(f"📊 Parsing SDF: {args.sdf}")

    # Parse obstacles
    ignore_names = [name.strip() for name in args.ignore.split(",") if name.strip()]
    obstacles = parse_sdf_obstacles(args.sdf, ignore_names)
    print(f"✓ Loaded {len(obstacles)} obstacles")

    # Auto-calculate grid range if not provided
    if args.grid_range is None:
        args.grid_range = calculate_grid_range(obstacles)
        print(f"  Range auto-calculated: ±{args.grid_range} m")
    else:
        print(f"  Range: ±{args.grid_range} m (manual)")

    # Build grid
    spec = GridSpec(size=args.grid_size, grid_range=args.grid_range)
    grid = build_occupancy_grid(spec, obstacles)
    print(f"✓ Built {spec.size}×{spec.size} grid")
    print(f"  Resolution: {spec.resolution:.3f} m/cell")
    print(f"  Occupied cells: {grid.occupied_count()}")

    # Visualize
    print("\n📈 Displaying visualization...")
    simple_plot(grid, obstacles, title=f"Occupancy Grid ({spec.size}×{spec.size})")


if __name__ == "__main__":
    main()


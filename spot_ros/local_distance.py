#!/usr/bin/env python3
"""Local obstacle distance field wrapper."""

from spot_ros.ObstacleGrid import ObstacleGrid


class LocalDistanceField:
    """Wraps ObstacleGrid for local obstacle queries using static SDF grid."""

    def __init__(self, static_cache):
        """Initialize from static SDF cache (loaded once at startup)."""
        self.static = static_cache
        # Build obstacle grid immediately from static cache
        self.obstacle_grid = self._build_grid_from_static()

    def _build_grid_from_static(self):
        """Build ObstacleGrid from static cache data."""
        if self.static is None:
            return None

        # Create ObstacleGrid using the static cache grid data
        return ObstacleGrid.from_grid_data(
            resolution=self.static.resolution,
            width=self.static.width,
            height=self.static.height,
            origin_x=self.static.origin_x,
            origin_y=self.static.origin_y,
            origin_yaw=0.0,  # Static grid has no yaw rotation
            data=self.static.grid_data,
            occupied_threshold=50,
            treat_unknown_as_obstacle=False,
        )

    def update_from_occupancy_msg(self, msg):
        """Update from /spot/local_grid ROS message (optional, for debugging)."""
        # Can be called but grid is primarily static
        pass

    def is_free(self, x, y, threshold=-0.1):
        """Check if point is in free space."""
        if self.obstacle_grid is None:
            return True  # Fallback: assume free if no grid
        dist = self.obstacle_grid.distance_at(x, y, out_of_map_value=0.5)
        return dist > threshold

    def distance_at(self, x, y):
        """Get signed distance at world point."""
        if self.obstacle_grid is None:
            return 0.5
        return self.obstacle_grid.distance_at(x, y)


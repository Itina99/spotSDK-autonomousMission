#!/usr/bin/env python3
import time
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from spot_ros.environmentMap import EnvironmentMap
from spot_ros import movements_ros
from spot_ros import spotUtils_ros
from spot_ros import spotGrid_ros
from spot_ros import navGraphUtils_ros
from spot_ros.local_distance import LocalDistanceField
from spot_ros.static_grid_loader import load_static_grid


ROS_NODE = None
GLOBAL_OBSTACLE_POINTS = {}
GLOBAL_PADDING_POINTS = {}
GLOBAL_FREE_POINTS = {}

ROBOT_PATH_HISTORY = []

# Global sets to track observed points across visualizations (persistent exploration map)
GLOBAL_OBSERVED_OBSTACLES = set()  # Set of tuples (mx, my, dist) to avoid duplicates
GLOBAL_OBSERVED_PADDING = set()
GLOBAL_OBSERVED_FREE = set()


def check_line_of_sight(x1, y1, x2, y2, pts, cells, obstacle_threshold=0.0):
    """
    Check if there's a clear line of sight between two points.
    Uses sampling along the line to check for obstacles.

    Uses the obstacle_distance grid where:
        dist < threshold -> blocked
        dist >= threshold -> passable

    Args:
        x1, y1: Start coordinates (robot position)
        x2, y2: End coordinates (target point)
        pts: Grid points array from obstacle_distance grid
        cells: obstacle_distance values per cell
        obstacle_threshold: Cells with distance strictly less than this value are
                            considered blocked. Default 0.0 = zero padding.

    Returns:
        bool: True if path is clear, False if blocked
    """
    distance = math.hypot(x2 - x1, y2 - y1)
    num_checks = max(10, int(distance * 10))

    # Convert pts to numpy array if needed (SDK-compatible)
    pts_np = np.asarray(pts)

    for i in range(num_checks):
        t = i / max(1, num_checks - 1)
        check_x = x1 + t * (x2 - x1)
        check_y = y1 + t * (y2 - y1)

        # Find nearest grid point
        distances = np.sqrt((pts_np[:, 0] - check_x) ** 2 + (pts_np[:, 1] - check_y) ** 2)
        nearest_idx = int(np.argmin(distances))

        # Blocked only when strictly inside an obstacle (dist < threshold)
        if cells[nearest_idx] < obstacle_threshold:
            return False

    return True

def get_pts_and_cells_from_local_distance(local_distance):
    """
    Extract pts (grid points) and cells (obstacle distances) from LocalDistanceField.

    This allows find_best_point_in_cell() to be called with the SDK-compatible signature.

    Args:
        local_distance: LocalDistanceField instance

    Returns:
        tuple: (pts, cells_obstacle_dist) compatible with SDK signature
        or (None, None) if local_distance is unavailable
    """
    if local_distance is None:
        return None, None

    if local_distance.obstacle_grid is None:
        return None, None

    # Use the to_points_and_cells() method of ObstacleGrid
    pts, cells = local_distance.obstacle_grid.to_points_and_cells()
    return pts, cells

def check_line_of_sight_static(x1, y1, x2, y2, local_distance):
    """
    Check if there's a clear line of sight between two points using static distance field only.

    Args:
        x1, y1: Start point
        x2, y2: End point
        local_distance: LocalDistanceField instance with static grid

    Returns:
        True if path is clear, False if blocked by obstacle
    """
    distance = math.hypot(x2 - x1, y2 - y1)
    num_checks = max(10, int(distance * 10))

    threshold = -0.15  # Same threshold as is_free() check

    for i in range(num_checks):
        t = i / max(1, num_checks - 1)
        check_x = x1 + t * (x2 - x1)
        check_y = y1 + t * (y2 - y1)

        # Check if this point in the path is free
        if not local_distance.is_free(check_x, check_y, threshold=threshold):
            return False

    return True

def sample_cell_points(env, cell_row, cell_col, num_samples=200):
    """Sample random points within a cell (ROS version)."""
    world_pos = env.get_world_position_from_cell(cell_row, cell_col)
    if world_pos is None:
        return []

    cell_center_x, cell_center_y = world_pos
    half_size = env.cell_size / 2.0

    samples = []
    for _ in range(num_samples):
        offset_x = np.random.uniform(-half_size * 0.8, half_size * 0.8)
        offset_y = np.random.uniform(-half_size * 0.8, half_size * 0.8)

        cos_yaw = math.cos(env.origin_yaw)
        sin_yaw = math.sin(env.origin_yaw)

        world_offset_x = offset_x * cos_yaw - offset_y * sin_yaw
        world_offset_y = offset_x * sin_yaw + offset_y * cos_yaw

        sample_x = cell_center_x + world_offset_x
        sample_y = cell_center_y + world_offset_y
        samples.append((sample_x, sample_y))

    return samples

def find_best_point_in_cell(robot_x, robot_y, env, cell_row, cell_col, pts, cells_obstacle_dist):
    """Find the closest valid point to the cell center with clear LOS.

    **IDENTICA ALLA VERSIONE SDK** - fully compatible signature.

    Args:
        robot_x, robot_y: Current robot position
        env: EnvironmentMap instance
        cell_row, cell_col: Target cell coordinates
        pts: Grid points array from obstacle_distance grid
        cells_obstacle_dist: Cell values from obstacle_distance grid

    Returns:
        tuple: (best_x, best_y, valid_samples, rejected_samples) or (None, None, [], [])
    """
    sampled_points = sample_cell_points(env, cell_row, cell_col, num_samples=100)
    if not sampled_points:
        return None, None, [], []

    cell_center = env.get_world_position_from_cell(cell_row, cell_col)
    if cell_center is None:
        return None, None, [], []

    cell_center_x, cell_center_y = cell_center
    valid_samples = []
    rejected_samples = []

    # Check each sampled point using check_line_of_sight (SDK-compatible)
    for sample_x, sample_y in sampled_points:
        if check_line_of_sight(robot_x, robot_y, sample_x, sample_y, pts, cells_obstacle_dist, obstacle_threshold=0.15):
            valid_samples.append((sample_x, sample_y))
        else:
            rejected_samples.append((sample_x, sample_y))

    if not valid_samples:
        print(f"[WARNING] No clear path found to any sampled point in cell ({cell_row},{cell_col})")
        return None, None, valid_samples, rejected_samples

    best_point = None
    min_distance = float('inf')
    for sample_x, sample_y in valid_samples:
        dist = math.hypot(sample_x - cell_center_x, sample_y - cell_center_y)
        if dist < min_distance:
            min_distance = dist
            best_point = (sample_x, sample_y)

    return best_point[0], best_point[1], valid_samples, rejected_samples

def draw_explored_sides(segments, cell_x, cell_y, half_size, sides_status, cos_yaw, sin_yaw):
    """Collect segments for explored cell sides (RViz line list)."""
    if sides_status == 0b0000:
        return

    edge_inset = 0.05

    def add_segment(start, end):
        sx, sy = start
        ex, ey = end
        wx1 = cell_x + (sx * cos_yaw - sy * sin_yaw)
        wy1 = cell_y + (sx * sin_yaw + sy * cos_yaw)
        wx2 = cell_x + (ex * cos_yaw - ey * sin_yaw)
        wy2 = cell_y + (ex * sin_yaw + ey * cos_yaw)
        segments.append(((wx1, wy1), (wx2, wy2)))

    if sides_status & 0b1000:
        add_segment((-half_size + edge_inset, half_size), (half_size - edge_inset, half_size))
    if sides_status & 0b0100:
        add_segment((half_size, -half_size + edge_inset), (half_size, half_size - edge_inset))
    if sides_status & 0b0010:
        add_segment((-half_size + edge_inset, -half_size), (half_size - edge_inset, -half_size))
    if sides_status & 0b0001:
        add_segment((-half_size, -half_size + edge_inset), (-half_size, half_size - edge_inset))

def visualize_grid_with_candidates(pts, cells_obstacle_dist, color, robot_x, robot_y,
                                   candidates, chosen_point, iteration, env=None, save_path=None):
    """Publish RViz markers mirroring the SDK matplotlib view."""
    node = ROS_NODE
    if node is None or not getattr(node, 'viz_enabled', False):
        return

    frame_id = node.map_frame if node.map_frame else 'map'
    stamp = node.get_clock().now().to_msg()
    marker_array = MarkerArray()

    def rgba(r, g, b, a=1.0):
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

    def pt(x, y, z=0.03):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def add_marker_fixed(ns, mid, mtype, color, sx, sy, sz, action=Marker.ADD):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = action
        m.pose.orientation.w = 1.0
        m.scale.x = float(sx)
        m.scale.y = float(sy)
        m.scale.z = float(sz)
        m.color = color
        marker_array.markers.append(m)
        return m

    local_radius = float(getattr(node, 'viz_local_radius', 2.0))
    padding_threshold = 0.15
    grid_point_scale = 0.08

    obstacle_points = []
    padding_points = []
    free_points = []

    # **OPTIMIZED**: Draw ONLY local grid within radius to avoid rendering huge maps
    if hasattr(pts, 'width') and hasattr(pts, 'height'):
        helper = pts
        # Calculate bounds in grid coordinates for local range
        origin_x = helper.origin_x
        origin_y = helper.origin_y
        res = helper.resolution

        # Convert world bounds to grid indices
        min_x_world = robot_x - local_radius
        max_x_world = robot_x + local_radius
        min_y_world = robot_y - local_radius
        max_y_world = robot_y + local_radius

        # Iterate only over local cells (not entire grid)
        for mx in range(helper.width):
            for my in range(helper.height):
                wx, wy = helper.map_to_world(mx, my)
                # Skip if outside local radius
                if wx < min_x_world or wx > max_x_world or wy < min_y_world or wy > max_y_world:
                    continue

                dist = cells_obstacle_dist[my * helper.width + mx]
                p = pt(wx, wy, 0.02)
                if dist < 0.0:
                    obstacle_points.append(p)
                elif dist < padding_threshold:
                    padding_points.append(p)
                else:
                    free_points.append(p)
    else:
        for (wx, wy), dist in zip(pts, cells_obstacle_dist):
            if abs(wx - robot_x) > local_radius or abs(wy - robot_y) > local_radius:
                continue
            p = pt(wx, wy, 0.02)
            if dist < 0.0:
                obstacle_points.append(p)
            elif dist < padding_threshold:
                padding_points.append(p)
            else:
                free_points.append(p)

    if obstacle_points:
        m = add_marker_fixed('grid_obstacle', 10, Marker.POINTS, rgba(1.0, 0.0, 0.0, 0.85), grid_point_scale, grid_point_scale, 0.01)
        m.points = obstacle_points
    else:
        add_marker_fixed('grid_obstacle', 10, Marker.POINTS, rgba(0.0, 0.0, 0.0, 0.0), grid_point_scale, grid_point_scale, 0.01, action=Marker.DELETE)
    if padding_points:
        m = add_marker_fixed('grid_padding', 11, Marker.POINTS, rgba(0.0, 1.0, 0.0, 0.75), grid_point_scale, grid_point_scale, 0.01)
        m.points = padding_points
    else:
        add_marker_fixed('grid_padding', 11, Marker.POINTS, rgba(0.0, 0.0, 0.0, 0.0), grid_point_scale, grid_point_scale, 0.01, action=Marker.DELETE)
    if free_points:
        m = add_marker_fixed('grid_free', 12, Marker.POINTS, rgba(0.1, 0.3, 1.0, 0.65), grid_point_scale, grid_point_scale, 0.01)
        m.points = free_points
    else:
        add_marker_fixed('grid_free', 12, Marker.POINTS, rgba(0.0, 0.0, 0.0, 0.0), grid_point_scale, grid_point_scale, 0.01, action=Marker.DELETE)

    if env is not None:
        cos_yaw = math.cos(env.origin_yaw)
        sin_yaw = math.sin(env.origin_yaw)
        half_size = env.cell_size / 2.0

        visited_lines = add_marker_fixed('cells_visited', 20, Marker.LINE_LIST, rgba(0.0, 0.5, 0.0, 0.85), 0.05, 0.0, 0.0)
        blocked_lines = add_marker_fixed('cells_blocked', 21, Marker.LINE_LIST, rgba(0.7, 0.0, 0.0, 0.85), 0.05, 0.0, 0.0)
        unvisited_lines = add_marker_fixed('cells_unvisited', 22, Marker.LINE_LIST, rgba(0.5, 0.5, 0.5, 0.55), 0.03, 0.0, 0.0)
        explored_lines = add_marker_fixed('cells_explored_sides', 23, Marker.LINE_LIST, rgba(1.0, 0.0, 0.0, 0.9), 0.05, 0.0, 0.0)

        explored_segments = []
        for row in range(env.rows):
            for col in range(env.cols):
                center = env.get_world_position_from_cell(row, col)
                if center is None:
                    continue
                cell_x, cell_y = center
                if abs(cell_x - robot_x) > (local_radius + env.cell_size) or abs(cell_y - robot_y) > (
                        local_radius + env.cell_size):
                    continue

                corners_grid = [
                    (-half_size, -half_size),
                    (half_size, -half_size),
                    (half_size, half_size),
                    (-half_size, half_size),
                ]
                corners = []
                for gx, gy in corners_grid:
                    wx = cell_x + (gx * cos_yaw - gy * sin_yaw)
                    wy = cell_y + (gx * sin_yaw + gy * cos_yaw)
                    corners.append(pt(wx, wy, 0.04))

                status_raw = env.get_cell_status(row, col)
                if isinstance(status_raw, tuple):
                    cell_status, sides_status = status_raw
                else:
                    cell_status = status_raw
                    sides_status = 0b0000

                target_lines = unvisited_lines
                if cell_status == 1:
                    target_lines = visited_lines
                elif cell_status == -1:
                    target_lines = blocked_lines

                for i in range(4):
                    target_lines.points.append(corners[i])
                    target_lines.points.append(corners[(i + 1) % 4])

                if sides_status != 0b0000:
                    draw_explored_sides(explored_segments, cell_x, cell_y, half_size, sides_status, cos_yaw, sin_yaw)

        if explored_segments:
            for (sx, sy), (ex, ey) in explored_segments:
                explored_lines.points.append(pt(sx, sy, 0.05))
                explored_lines.points.append(pt(ex, ey, 0.05))
    else:
        add_marker_fixed('local_cells_visited', 6, Marker.LINE_LIST, rgba(0.0, 0.0, 0.0, 0.0), 0.05, 0.0, 0.0, action=Marker.DELETE)
        add_marker_fixed('local_cells_blocked', 7, Marker.LINE_LIST, rgba(0.0, 0.0, 0.0, 0.0), 0.05, 0.0, 0.0, action=Marker.DELETE)
        add_marker_fixed('local_cells_unvisited', 8, Marker.LINE_LIST, rgba(0.0, 0.0, 0.0, 0.0), 0.03, 0.0, 0.0, action=Marker.DELETE)
        add_marker_fixed('local_cells_explored_sides', 9, Marker.LINE_LIST, rgba(0.0, 0.0, 0.0, 0.0), 0.05, 0.0, 0.0, action=Marker.DELETE)

    rejected = []
    valid = []
    if isinstance(candidates, dict):
        rejected = list(candidates.get('rejected', []))
        valid = list(candidates.get('valid', []))

    if rejected:
        m = add_marker_fixed('candidates_rejected', 30, Marker.POINTS, rgba(1.0, 0.0, 0.0, 1.0), 0.14, 0.14, 0.01)
        m.points = [pt(px, py, 0.06) for px, py in rejected]
    else:
        add_marker_fixed('candidates_rejected', 30, Marker.POINTS, rgba(0.0, 0.0, 0.0, 0.0), 0.14, 0.14, 0.01, action=Marker.DELETE)

    if valid:
        m = add_marker_fixed('candidates_valid', 31, Marker.POINTS, rgba(1.0, 0.9, 0.0, 0.9), 0.12, 0.12, 0.01)
        m.points = [pt(px, py, 0.06) for px, py in valid]
    else:
        add_marker_fixed('candidates_valid', 31, Marker.POINTS, rgba(0.0, 0.0, 0.0, 0.0), 0.12, 0.12, 0.01, action=Marker.DELETE)

    # **NEW**: Draw robot exploration path history
    global ROBOT_PATH_HISTORY
    # Add current position to path if it's far enough from the last recorded position
    if len(ROBOT_PATH_HISTORY) == 0 or math.hypot(robot_x - ROBOT_PATH_HISTORY[-1][0], robot_y - ROBOT_PATH_HISTORY[-1][1]) > 0.1:
        ROBOT_PATH_HISTORY.append((robot_x, robot_y))

    # Draw path as a line strip
    if len(ROBOT_PATH_HISTORY) > 1:
        path_line = add_marker_fixed('robot_path', 40, Marker.LINE_STRIP, rgba(0.2, 0.8, 0.2, 0.8), 0.04, 0.0, 0.0)
        path_line.points = [pt(px, py, 0.03) for px, py in ROBOT_PATH_HISTORY[-50:]]  # Last 50 points to avoid huge marker
    else:
        add_marker_fixed('robot_path', 40, Marker.LINE_STRIP, rgba(0.0, 0.0, 0.0, 0.0), 0.04, 0.0, 0.0, action=Marker.DELETE)

    robot_marker = add_marker_fixed('robot_pose', 41, Marker.SPHERE, rgba(0.0, 0.1, 1.0, 0.95), 0.28, 0.28, 0.16)
    robot_marker.pose.position = pt(robot_x, robot_y, 0.10)

    if chosen_point is not None:
        tx, ty = chosen_point
        target = add_marker_fixed('chosen_target', 42, Marker.SPHERE, rgba(0.0, 0.9, 0.0, 1.0), 0.30, 0.30, 0.20)
        target.pose.position = pt(tx, ty, 0.10)

        target_line = add_marker_fixed('target_segment', 43, Marker.LINE_STRIP, rgba(0.0, 0.8, 0.0, 0.95), 0.05, 0.0, 0.0)
        target_line.points = [pt(robot_x, robot_y, 0.05), pt(tx, ty, 0.05)]

        dist = math.hypot(tx - robot_x, ty - robot_y)
        label = add_marker_fixed('target_distance', 44, Marker.TEXT_VIEW_FACING, rgba(0.0, 0.4, 0.0, 1.0), 0.0, 0.0, 0.22)
        label.pose.position = pt((robot_x + tx) / 2.0, (robot_y + ty) / 2.0, 0.22)
        label.text = f"{dist:.2f}m"
    else:
        add_marker_fixed('chosen_target', 42, Marker.SPHERE, rgba(0.0, 0.0, 0.0, 0.0), 0.30, 0.30, 0.20, action=Marker.DELETE)
        add_marker_fixed('target_segment', 43, Marker.LINE_STRIP, rgba(0.0, 0.0, 0.0, 0.0), 0.05, 0.0, 0.0, action=Marker.DELETE)
        add_marker_fixed('target_distance', 44, Marker.TEXT_VIEW_FACING, rgba(0.0, 0.0, 0.0, 0.0), 0.0, 0.0, 0.22, action=Marker.DELETE)

    info = add_marker_fixed('iteration_info', 45, Marker.TEXT_VIEW_FACING, rgba(0.1, 0.1, 0.1, 0.95), 0.0, 0.0, 0.24)
    info.pose.position = pt(robot_x, robot_y + 0.45, 0.28)
    info.text = f"iter={iteration} valid={len(valid)} rejected={len(rejected)}"

    node.viz_pub.publish(marker_array)

# Persistent visualization limits
MAX_PERSISTENT_POINTS = 50000
PERSISTENT_UPDATE_RATE = 20
PERSISTENT_DOWNSAMPLE = 2


def visualize_grid_static(local_distance, robot_x, robot_y, candidates, chosen_point, iteration, env=None):
    """Publish RViz markers using the static grid and optionally SLAM data.

    Optimizations:
    - Limited FOV rendering
    - Persistent history rendered less frequently
    - Downsampling for RViz performance
    - Reduced marker rebuild overhead
    """

    global GLOBAL_OBSERVED_OBSTACLES
    global GLOBAL_OBSERVED_PADDING
    global GLOBAL_OBSERVED_FREE
    global ROBOT_PATH_HISTORY

    node = ROS_NODE
    if node is None or not getattr(node, 'viz_enabled', False):
        return

    viz_start_time = time.time()

    frame_id = 'map'
    stamp = node.get_clock().now().to_msg()
    marker_array = MarkerArray()

    def rgba(r, g, b, a=1.0):
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

    def pt(x, y, z=0.03):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def add_marker_fixed(ns, mid, mtype, color, sx, sy, sz, action=Marker.ADD):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = action
        m.pose.orientation.w = 1.0
        m.scale.x = float(sx)
        m.scale.y = float(sy)
        m.scale.z = float(sz)
        m.color = color
        marker_array.markers.append(m)
        return m

    # ------------------------------------------------------------------
    # Global explored map
    # ------------------------------------------------------------------

    prof_global_start = time.time()

    if env is not None:

        cos_yaw = math.cos(env.origin_yaw)
        sin_yaw = math.sin(env.origin_yaw)
        half_size = env.cell_size / 2.0

        global_visited_bg = add_marker_fixed(
            'persistent_global_visited',
            0,
            Marker.LINE_LIST,
            rgba(0.0, 0.5, 0.0, 0.25),
            0.03,
            0.0,
            0.0
        )

        global_blocked_bg = add_marker_fixed(
            'persistent_global_blocked',
            1,
            Marker.LINE_LIST,
            rgba(0.7, 0.0, 0.0, 0.25),
            0.03,
            0.0,
            0.0
        )

        for row in range(env.rows):
            for col in range(env.cols):

                center = env.get_world_position_from_cell(row, col)
                if center is None:
                    continue

                cell_x, cell_y = center

                status_raw = env.get_cell_status(row, col)

                if isinstance(status_raw, tuple):
                    cell_status, _ = status_raw
                else:
                    cell_status = status_raw

                if cell_status == 0:
                    continue

                corners_grid = [
                    (-half_size, -half_size),
                    (half_size, -half_size),
                    (half_size, half_size),
                    (-half_size, half_size),
                ]

                corners = []

                for gx, gy in corners_grid:
                    wx = cell_x + (gx * cos_yaw - gy * sin_yaw)
                    wy = cell_y + (gx * sin_yaw + gy * cos_yaw)
                    corners.append(pt(wx, wy, 0.00))

                target_bg = (
                    global_visited_bg
                    if cell_status == 1
                    else global_blocked_bg
                )

                for i in range(4):
                    target_bg.points.append(corners[i])
                    target_bg.points.append(corners[(i + 1) % 4])

    prof_global_time = time.time() - prof_global_start

    # ------------------------------------------------------------------
    # Robot path history
    # ------------------------------------------------------------------

    if len(ROBOT_PATH_HISTORY) > 1:

        path_line = add_marker_fixed(
            'persistent_robot_path',
            2,
            Marker.LINE_STRIP,
            rgba(0.2, 0.8, 0.2, 0.7),
            0.05,
            0.0,
            0.0
        )

        path_line.points = [
            pt(px, py, 0.005)
            for px, py in ROBOT_PATH_HISTORY
        ]
    else:
        add_marker_fixed(
            'persistent_robot_path',
            2,
            Marker.LINE_STRIP,
            rgba(0.0, 0.0, 0.0, 0.0),
            0.05,
            0.0,
            0.0,
            action=Marker.DELETE
        )

    # ------------------------------------------------------------------
    # Clear local markers only (NOT the persistent path!)
    # ------------------------------------------------------------------
    # IMPORTANT: We DON'T use DELETEALL here because it would erase
    # the persistent_robot_path. Instead, we'll overwrite local markers
    # with new data by using consistent IDs within their namespace.

    # ------------------------------------------------------------------
    # Local FOV grid
    # ------------------------------------------------------------------

    local_radius = 2.0
    padding_threshold = 0.15

    prof_sdf_start = time.time()

    if (
        local_distance is not None and
        hasattr(local_distance, 'obstacle_grid') and
        local_distance.obstacle_grid is not None
    ):

        spec = local_distance.obstacle_grid.spec

        obstacle_cubes = add_marker_fixed(
            'local_grid_obstacle_sdf',
            3,
            Marker.CUBE_LIST,
            rgba(1.0, 0.0, 0.0, 0.35),
            spec.resolution,
            spec.resolution,
            0.01
        )

        padding_cubes = add_marker_fixed(
            'local_grid_padding_sdf',
            4,
            Marker.CUBE_LIST,
            rgba(1.0, 1.0, 0.0, 0.35),
            spec.resolution,
            spec.resolution,
            0.01
        )

        free_cubes = add_marker_fixed(
            'local_grid_free_sdf',
            5,
            Marker.CUBE_LIST,
            rgba(0.2, 0.8, 0.2, 0.35),
            spec.resolution,
            spec.resolution,
            0.01
        )

        # IMPORTANT: Clear points from previous frames before adding new ones
        obstacle_cubes.points = []
        padding_cubes.points = []
        free_cubes.points = []

        for mx in range(spec.width):
            for my in range(spec.height):

                wx, wy = spec.map_to_world(mx, my)

                if (
                    abs(wx - robot_x) > local_radius or
                    abs(wy - robot_y) > local_radius
                ):
                    continue

                dist = local_distance.obstacle_grid.signed_cells[
                    my * spec.width + mx
                ]

                p = pt(wx, wy, 0.01)
                point_key = (
                    round(wx, 2),
                    round(wy, 2),
                    round(dist, 3)
                )

                if dist < 0.0:
                    obstacle_cubes.points.append(p)
                    GLOBAL_OBSERVED_OBSTACLES.add(point_key)

                    if len(GLOBAL_OBSERVED_OBSTACLES) > MAX_PERSISTENT_POINTS:
                        GLOBAL_OBSERVED_OBSTACLES.pop()

                elif dist < padding_threshold:
                    padding_cubes.points.append(p)
                    GLOBAL_OBSERVED_PADDING.add(point_key)

                    if len(GLOBAL_OBSERVED_PADDING) > MAX_PERSISTENT_POINTS:
                        GLOBAL_OBSERVED_PADDING.pop()

                else:
                    free_cubes.points.append(p)
                    GLOBAL_OBSERVED_FREE.add(point_key)

                    if len(GLOBAL_OBSERVED_FREE) > MAX_PERSISTENT_POINTS:
                        GLOBAL_OBSERVED_FREE.pop()


    prof_sdf_time = time.time() - prof_sdf_start

    # ------------------------------------------------------------------
    # Environment cells
    # ------------------------------------------------------------------

    prof_env_start = time.time()

    if env is not None:

        cos_yaw = math.cos(env.origin_yaw)
        sin_yaw = math.sin(env.origin_yaw)
        half_size = env.cell_size / 2.0

        visited_lines = add_marker_fixed(
            'local_cells_visited',
            6,
            Marker.LINE_LIST,
            rgba(0.0, 0.5, 0.0, 0.85),
            0.05,
            0.0,
            0.0
        )

        blocked_lines = add_marker_fixed(
            'local_cells_blocked',
            7,
            Marker.LINE_LIST,
            rgba(0.7, 0.0, 0.0, 0.85),
            0.05,
            0.0,
            0.0
        )

        unvisited_lines = add_marker_fixed(
            'local_cells_unvisited',
            8,
            Marker.LINE_LIST,
            rgba(0.5, 0.5, 0.5, 0.55),
            0.03,
            0.0,
            0.0
        )

        explored_lines = add_marker_fixed(
            'local_cells_explored_sides',
            9,
            Marker.LINE_LIST,
            rgba(1.0, 0.0, 0.0, 0.9),
            0.05,
            0.0,
            0.0
        )

        explored_segments = []

        for row in range(env.rows):
            for col in range(env.cols):

                center = env.get_world_position_from_cell(row, col)
                if center is None:
                    continue

                cell_x, cell_y = center

                if (
                    abs(cell_x - robot_x) > local_radius or
                    abs(cell_y - robot_y) > local_radius
                ):
                    continue

                corners_grid = [
                    (-half_size, -half_size),
                    (half_size, -half_size),
                    (half_size, half_size),
                    (-half_size, half_size),
                ]

                corners = []

                for gx, gy in corners_grid:
                    wx = cell_x + (gx * cos_yaw - gy * sin_yaw)
                    wy = cell_y + (gx * sin_yaw + gy * cos_yaw)
                    corners.append(pt(wx, wy, 0.04))

                status_raw = env.get_cell_status(row, col)

                if isinstance(status_raw, tuple):
                    cell_status, sides_status = status_raw
                else:
                    cell_status = status_raw
                    sides_status = 0b0000

                target_lines = unvisited_lines

                if cell_status == 1:
                    target_lines = visited_lines
                elif cell_status == -1:
                    target_lines = blocked_lines

                for i in range(4):
                    target_lines.points.append(corners[i])
                    target_lines.points.append(corners[(i + 1) % 4])

                if sides_status != 0b0000:
                    draw_explored_sides(
                        explored_segments,
                        cell_x,
                        cell_y,
                        half_size,
                        sides_status,
                        cos_yaw,
                        sin_yaw
                    )

        if explored_segments:
            for (sx, sy), (ex, ey) in explored_segments:
                explored_lines.points.append(pt(sx, sy, 0.05))
                explored_lines.points.append(pt(ex, ey, 0.05))
    else:
        add_marker_fixed('local_cells_visited', 6, Marker.LINE_LIST, rgba(0.0, 0.0, 0.0, 0.0), 0.05, 0.0, 0.0, action=Marker.DELETE)
        add_marker_fixed('local_cells_blocked', 7, Marker.LINE_LIST, rgba(0.0, 0.0, 0.0, 0.0), 0.05, 0.0, 0.0, action=Marker.DELETE)
        add_marker_fixed('local_cells_unvisited', 8, Marker.LINE_LIST, rgba(0.0, 0.0, 0.0, 0.0), 0.03, 0.0, 0.0, action=Marker.DELETE)
        add_marker_fixed('local_cells_explored_sides', 9, Marker.LINE_LIST, rgba(0.0, 0.0, 0.0, 0.0), 0.05, 0.0, 0.0, action=Marker.DELETE)

    prof_env_time = time.time() - prof_env_start

    # ------------------------------------------------------------------
    # Persistent cumulative observed grid
    # ------------------------------------------------------------------

    prof_cumulative_start = time.time()

    persistent_resolution = (
        local_distance.obstacle_grid.spec.resolution
        if (
                local_distance is not None and
                hasattr(local_distance, 'obstacle_grid') and
                local_distance.obstacle_grid is not None
        )
        else 0.1
    )

    # ------------------------------------------------------------------
    # Obstacles
    # ------------------------------------------------------------------

    if GLOBAL_OBSERVED_OBSTACLES:
        persistent_obstacle_cubes = add_marker_fixed(
            'persistent_observed_obstacles',
            10,
            Marker.CUBE_LIST,
            rgba(1.0, 0.0, 0.0, 0.35),  # transparent
            persistent_resolution,
            persistent_resolution,
            0.01
        )

        persistent_obstacle_cubes.points = [
            pt(wx, wy, 0.0)
            for wx, wy, _ in GLOBAL_OBSERVED_OBSTACLES
        ]
    else:
        add_marker_fixed('persistent_observed_obstacles', 10, Marker.CUBE_LIST, rgba(0.0, 0.0, 0.0, 0.0), persistent_resolution, persistent_resolution, 0.01, action=Marker.DELETE)

    # ------------------------------------------------------------------
    # Padding
    # ------------------------------------------------------------------

    if GLOBAL_OBSERVED_PADDING:
        persistent_padding_cubes = add_marker_fixed(
            'persistent_observed_padding',
            11,
            Marker.CUBE_LIST,
            rgba(1.0, 1.0, 0.0, 0.35),
            persistent_resolution,
            persistent_resolution,
            0.01
        )

        persistent_padding_cubes.points = [
            pt(wx, wy, 0.0)
            for wx, wy, _ in GLOBAL_OBSERVED_PADDING
        ]
    else:
        add_marker_fixed('persistent_observed_padding', 11, Marker.CUBE_LIST, rgba(0.0, 0.0, 0.0, 0.0), persistent_resolution, persistent_resolution, 0.01, action=Marker.DELETE)

    # ------------------------------------------------------------------
    # Free space
    # ------------------------------------------------------------------

    if GLOBAL_OBSERVED_FREE:
        persistent_free_cubes = add_marker_fixed(
            'persistent_observed_free',
            12,
            Marker.CUBE_LIST,
            rgba(0.2, 0.8, 0.2, 0.35),
            persistent_resolution,
            persistent_resolution,
            0.01
        )

        persistent_free_cubes.points = [
            pt(wx, wy, 0.0)
            for wx, wy, _ in GLOBAL_OBSERVED_FREE
        ]
    else:
        add_marker_fixed('persistent_observed_free', 12, Marker.CUBE_LIST, rgba(0.0, 0.0, 0.0, 0.0), persistent_resolution, persistent_resolution, 0.01, action=Marker.DELETE)

    prof_cumulative_time = time.time() - prof_cumulative_start

    # ------------------------------------------------------------------
    # Candidates
    # ------------------------------------------------------------------

    prof_cand_start = time.time()

    rejected = []
    valid = []

    if isinstance(candidates, dict):
        rejected = list(candidates.get('rejected', []))
        valid = list(candidates.get('valid', []))

    if rejected:

        m = add_marker_fixed(
            'local_candidates_rejected',
            30,
            Marker.POINTS,
            rgba(1.0, 0.0, 0.0, 1.0),
            0.14,
            0.14,
            0.01
        )

        m.points = [
            pt(px, py, 0.06)
            for px, py in rejected
        ]
    else:
        add_marker_fixed('local_candidates_rejected', 30, Marker.POINTS, rgba(0.0, 0.0, 0.0, 0.0), 0.14, 0.14, 0.01, action=Marker.DELETE)

    if valid:

        m = add_marker_fixed(
            'local_candidates_valid',
            31,
            Marker.POINTS,
            rgba(1.0, 0.9, 0.0, 0.9),
            0.12,
            0.12,
            0.01
        )

        m.points = [
            pt(px, py, 0.06)
            for px, py in valid
        ]
    else:
        add_marker_fixed('local_candidates_valid', 31, Marker.POINTS, rgba(0.0, 0.0, 0.0, 0.0), 0.12, 0.12, 0.01, action=Marker.DELETE)

    # ------------------------------------------------------------------
    # Chosen point (green circle) and waypoint path
    # ------------------------------------------------------------------

    if chosen_point is not None:
        tx, ty = chosen_point
        # Visualize chosen point as a small green sphere (pallino)
        chosen_sphere = add_marker_fixed(
            'chosen_target_static',
            50,
            Marker.SPHERE,
            rgba(0.0, 1.0, 0.0, 1.0),  # Green
            0.15,  # Diameter: 15cm
            0.15,
            0.15
        )
        chosen_sphere.pose.position = pt(tx, ty, 0.10)

        # Draw line from robot to chosen point
        target_line = add_marker_fixed(
            'target_line_static',
            51,
            Marker.LINE_STRIP,
            rgba(0.0, 1.0, 0.0, 0.9),
            0.03,  # Line width
            0.0,
            0.0
        )
        target_line.points = [pt(robot_x, robot_y, 0.05), pt(tx, ty, 0.05)]
    else:
        add_marker_fixed('chosen_target_static', 50, Marker.SPHERE, rgba(0.0, 0.0, 0.0, 0.0), 0.15, 0.15, 0.15, action=Marker.DELETE)
        add_marker_fixed('target_line_static', 51, Marker.LINE_STRIP, rgba(0.0, 0.0, 0.0, 0.0), 0.03, 0.0, 0.0, action=Marker.DELETE)

    # Draw waypoint path
    if env is not None and hasattr(env, 'waypoints') and len(env.waypoints) > 1:
        waypoint_path = add_marker_fixed(
            'waypoint_path',
            52,
            Marker.LINE_STRIP,
            rgba(0.0, 0.7, 1.0, 0.8),  # Light blue
            0.06,  # Line width
            0.0,
            0.0
        )
        waypoint_path.points = [pt(wx, wy, 0.08) for wx, wy in env.waypoints]
    else:
        add_marker_fixed('waypoint_path', 52, Marker.LINE_STRIP, rgba(0.0, 0.0, 0.0, 0.0), 0.06, 0.0, 0.0, action=Marker.DELETE)

    # Draw waypoint markers (blue dots at each waypoint)
    if env is not None and hasattr(env, 'waypoints') and len(env.waypoints) > 0:
        waypoint_markers = add_marker_fixed(
            'waypoint_points',
            53,
            Marker.SPHERE_LIST,
            rgba(0.0, 0.0, 1.0, 0.95),  # Blue
            0.12,  # Diameter: 12cm
            0.12,
            0.12
        )
        waypoint_markers.points = [pt(wx, wy, 0.08) for wx, wy in env.waypoints]
    else:
        add_marker_fixed('waypoint_points', 53, Marker.SPHERE_LIST, rgba(0.0, 0.0, 0.0, 0.0), 0.12, 0.12, 0.12, action=Marker.DELETE)

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    prof_pub_start = time.time()

    node.viz_pub.publish(marker_array)

    prof_pub_time = time.time() - prof_pub_start

    viz_total_time = time.time() - viz_start_time

    # ------------------------------------------------------------------
    # Profiling logs
    # ------------------------------------------------------------------

    if iteration % 10 == 0:

        node.get_logger().info(
            f"[VIZ PROFILE] iter={iteration} | "
            f"global={prof_global_time*1000:.2f}ms "
            f"sdf={prof_sdf_time*1000:.2f}ms "
            f"env={prof_env_time*1000:.2f}ms "
            f"cumul={prof_cumulative_time*1000:.2f}ms "
            f"cand={prof_env_time*1000:.2f}ms "
            f"pub={prof_pub_time*1000:.2f}ms "
            f"TOTAL={viz_total_time*1000:.2f}ms "
            f"markers={len(marker_array.markers)} "
            f"persistent_pts=("
            f"obs={len(GLOBAL_OBSERVED_OBSTACLES)}, "
            f"pad={len(GLOBAL_OBSERVED_PADDING)}, "
            f"free={len(GLOBAL_OBSERVED_FREE)})"
        )


def attempt_enter_cell_from_position(node, env, target_row, target_col, mission_folder=None, iteration=0,
                                     recordingInterface=None):
    """
    Attempt to enter a target cell from the current robot position.
    Uses static SDF grid for obstacle avoidance - fully SDK-compatible.
    """
    cycle_start = time.time()

    # Verify SDF static grid is available
    if node.local_distance is None:
        node.get_logger().error('[ATTEMPT] Static SDF grid not available - cannot proceed.')
        return False

    # Get current robot position
    robot_x, robot_y, _, _ = spotUtils_ros.getPosition(node.pose_state)

    # Extract pts and cells from local_distance (SDK-compatible)
    pts, cells_obstacle_dist = get_pts_and_cells_from_local_distance(node.local_distance)

    if pts is None or cells_obstacle_dist is None:
        node.get_logger().error('[ATTEMPT] Could not extract grid data from local_distance.')
        return False

    # Find best point in target cell using EXACT SDK signature
    prof_best_point_start = time.time()
    target_x, target_y, valid_samples, rejected_samples = find_best_point_in_cell(
        robot_x, robot_y, env, target_row, target_col,
        pts,  # Grid points
        cells_obstacle_dist  # Obstacle distances
    )
    prof_best_point_time = time.time() - prof_best_point_start

    # Handle case where no valid point found
    if target_x is None or target_y is None:
        node.get_logger().warn(
            f"[ATTEMPT] Failed to find clear path to cell({target_row},{target_col}) | "
            f"best_pt={prof_best_point_time * 1000:.2f}ms"
        )

        # Visualize rejection
        prof_viz_start = time.time()
        visualize_grid_with_candidates(
            pts, cells_obstacle_dist, None, robot_x, robot_y,
            {'rejected': rejected_samples, 'valid': []},
            None,
            iteration,
            env=env
        )
        prof_viz_time = time.time() - prof_viz_start

        cycle_time = time.time() - cycle_start
        node.get_logger().info(
            f"[CYCLE PROFILE] cell({target_row},{target_col}) REJECTED | "
            f"best_pt={prof_best_point_time * 1000:.1f}ms viz={prof_viz_time * 1000:.1f}ms "
            f"TOTAL={cycle_time * 1000:.0f}ms"
        )

        return False

    # Visualize valid target
    prof_viz_start = time.time()
    visualize_grid_with_candidates(
        pts, cells_obstacle_dist, None, robot_x, robot_y,
        {'rejected': rejected_samples, 'valid': valid_samples},
        (target_x, target_y),
        iteration,
        env=env
    )
    prof_viz_time = time.time() - prof_viz_start

    # Calculate target orientation
    dx = target_x - robot_x
    dy = target_y - robot_y
    target_yaw = math.atan2(dy, dx)
    current_yaw = node.pose_state.yaw()
    dyaw = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))

    # Execute motion
    prof_motion_start = time.time()
    if not node.motion.rotate_by(dyaw):
        return False

    if not node.motion.move_to(target_x, target_y):
        return False

    prof_motion_time = time.time() - prof_motion_start

    # Verify arrival at target cell
    x_final, y_final, _, _ = spotUtils_ros.getPosition(node.pose_state)
    result = env.is_point_in_cell(x_final, y_final, target_row, target_col)

    # Performance report
    cycle_time = time.time() - cycle_start
    status = 'SUCCESS' if result else 'FAIL'
    node.get_logger().info(
        f"[CYCLE PROFILE] cell({target_row},{target_col}) {status} | "
        f"best_pt={prof_best_point_time * 1000:.1f}ms viz={prof_viz_time * 1000:.1f}ms "
        f"motion={prof_motion_time * 1000:.0f}ms TOTAL={cycle_time * 1000:.0f}ms"
    )

    return result
                               
def reset_persistent_exploration_map():
    """Reset the global exploration map (useful for starting new exploration sessions)."""
    global GLOBAL_OBSERVED_OBSTACLES, GLOBAL_OBSERVED_PADDING, GLOBAL_OBSERVED_FREE
    GLOBAL_OBSERVED_OBSTACLES.clear()
    GLOBAL_OBSERVED_PADDING.clear()
    GLOBAL_OBSERVED_FREE.clear()

def find_new_borders(env, robot_row, robot_col, path, frontier):
    new_borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
    new_borders_cells = []
    if len(new_borders) != 0:
        for new_border in new_borders:
            if new_border not in frontier and env.is_cell_visited(new_border[0], new_border[1]) != 1:
                new_borders_cells.append(new_border)
    return new_borders_cells

######## IL NODO EASY WALK VA GESTITO ESTERNAMENTE OPPURE GESTITO VIA FLAG #######
class EasyWalkROSNode(Node):
    def __init__(self):
        super().__init__('easy_walk_ros')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('viz_topic', '/easy_walk/visualization')
        self.declare_parameter('viz_enabled', True)
        self.declare_parameter('viz_local_radius', 2.0)
        self.declare_parameter('grid_rows', 4)
        self.declare_parameter('grid_cols', 4)
        self.declare_parameter('cell_size', 2.0)
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('yaw_tolerance', 0.08)
        self.declare_parameter('pos_tolerance', 0.06)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('timeout_scale', 4.0)
        self.declare_parameter('no_progress_timeout', 8.0)
        self.declare_parameter('progress_epsilon', 0.003)
        self.declare_parameter('linear_kp', 0.8)
        self.declare_parameter('angular_kp', 0.75)
        self.declare_parameter('max_angular_accel', 1.2)
        self.declare_parameter('move_yaw_blend_start', 0.35)
        self.declare_parameter('move_yaw_blend_stop', 1.2)
        self.declare_parameter('near_goal_radius', 0.15)
        self.declare_parameter('near_goal_timeout_boost', 2.0)
        self.declare_parameter('near_goal_yaw_relax', 0.35)
        self.declare_parameter('moving_angular_scale', 0.4)
        self.declare_parameter('moving_angular_cap', 0.25)
        self.declare_parameter('occupied_threshold', 65)

        self.pose_state = spotUtils_ros.PoseState()
        self.last_odom_stamp = None
        self.current_map = None
        self.map_frame = None
        self.viz_enabled = bool(self.get_parameter('viz_enabled').value)
        self.viz_local_radius = float(self.get_parameter('viz_local_radius').value)
        self.visualization_counter = 0

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        map_topic = self.get_parameter('map_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        viz_topic = self.get_parameter('viz_topic').value

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.viz_pub = self.create_publisher(MarkerArray, viz_topic, 1)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 1)

        self.motion = movements_ros.MotionController(
            get_pose_fn=self._get_pose,
            publish_cmd_fn=self._publish_cmd,
            linear_speed=self.get_parameter('linear_speed').value,
            angular_speed=self.get_parameter('angular_speed').value,
            yaw_tolerance=float(self.get_parameter('yaw_tolerance').value),
            pos_tolerance=float(self.get_parameter('pos_tolerance').value),
            spin_once_fn=self._spin_for_control,
            now_fn=self._now_seconds,
            control_period=float(self.get_parameter('control_period').value),
            timeout_scale=float(self.get_parameter('timeout_scale').value),
            no_progress_timeout=float(self.get_parameter('no_progress_timeout').value),
            progress_epsilon=float(self.get_parameter('progress_epsilon').value),
            linear_kp=float(self.get_parameter('linear_kp').value),
            angular_kp=float(self.get_parameter('angular_kp').value),
            max_angular_accel=float(self.get_parameter('max_angular_accel').value),
            move_yaw_blend_start=float(self.get_parameter('move_yaw_blend_start').value),
            move_yaw_blend_stop=float(self.get_parameter('move_yaw_blend_stop').value),
            near_goal_radius=float(self.get_parameter('near_goal_radius').value),
            near_goal_timeout_boost=float(self.get_parameter('near_goal_timeout_boost').value),
            near_goal_yaw_relax=float(self.get_parameter('near_goal_yaw_relax').value),
            moving_angular_scale=float(self.get_parameter('moving_angular_scale').value),
            moving_angular_cap=float(self.get_parameter('moving_angular_cap').value),
        )
        
        # Initialize static grid from SDF (for LOS checking - BACKUP/VERIFICATION)
        try:
            static_cache = load_static_grid('worlds/test.sdf')
            self.local_distance = LocalDistanceField(static_cache)
            self.get_logger().info('[EasyWalkROS] Static grid loaded from SDF')
        except Exception as e:
            self.get_logger().warn(f'[EasyWalkROS] Static grid load failed (will use SLAM): {e}')
            self.local_distance = None

    def _on_odom(self, msg: Odometry):
        spotUtils_ros.update_pose_from_odom(self.pose_state, msg)
        self.last_odom_stamp = Time.from_msg(msg.header.stamp)

    def _on_map(self, msg: OccupancyGrid):
        self.current_map = msg
        self.map_frame = msg.header.frame_id.lstrip('/') or msg.header.frame_id


    def _publish_cmd(self, cmd: Twist):
        self.cmd_pub.publish(cmd)

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _spin_for_control(self, timeout_sec=0.0):
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _get_pose(self):
        return self.pose_state.x, self.pose_state.y, self.pose_state.z, self.pose_state.yaw()

    def wait_for_data(self, timeout=10.0):
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            if self.current_map is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def easy_walk(options=None):
    ############## questa roba è tutta al posto dell'estop ecc gestire con flag le differenze ###########
    rclpy.init()
    node = EasyWalkROSNode()
    global ROS_NODE
    ROS_NODE = node

    # Reset persistent exploration map at the start of new exploration
    reset_persistent_exploration_map()

    try:
        if not node.wait_for_data():
            node.get_logger().error('Timeout waiting for map data.')
            return False

        env = EnvironmentMap(
            rows=node.get_parameter('grid_rows').value,
            cols=node.get_parameter('grid_cols').value,
            cell_size=node.get_parameter('cell_size').value,
        )
        #### In tutta la parte di setup c'è da controllare cosa aggiungere per allineare con la versione SDK, tipo cartella missione, registrazione waypoint, ecc #####
        x_boot, y_boot, z_boot, _ = spotUtils_ros.getPosition(node.pose_state)
        yaw_boot = node.pose_state.yaw()
        env.set_origin(x_boot, y_boot, yaw_boot, start_row=0, start_col=0)

        recording = navGraphUtils_ros.RecordingInterface()
        recording.clear_map()
        recording.create_default_waypoint(cell_row=0, cell_col=0, x=x_boot, y=y_boot, z=z_boot, yaw=yaw_boot)
        env.add_waypoint(x_boot, y_boot)
        env.add_robot_position(x_boot, y_boot, movement_type='explore')

        path = env.generate_serpentine_path(start_cell=env.start_cell)
        frontier = []

        robot_row, robot_col = env.get_cell_from_world(x_boot, y_boot)
        frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))

        while rclpy.ok():
            if len(frontier) == 0:
                break

            x, y, _, _ = spotUtils_ros.getPosition(node.pose_state)
            robot_row, robot_col = env.get_cell_from_world(x, y)

            borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
            borders_in_frontier = []
            for border in borders:
                if any((f[0] == border[0] and f[1] == border[1]) for f in frontier):
                    borders_in_frontier.append(border)

            if len(borders_in_frontier) != 0:
                selected_border = min(borders_in_frontier, key=lambda b: b[2])
                ####TUTTO OK MA VIENE PASSATO NODE INVECE DI LOCAL GRID CLIENT. FARE IL SOSTITUTO CLIENT#####
                ok = attempt_enter_cell_from_position(
                    node, env, selected_border[0], selected_border[1],
                    iteration=node.visualization_counter,
                    recordingInterface=recording,
                )
                node.visualization_counter += 1
                frontier.remove(selected_border)

                if ok:
                    x_new, y_new, z_new, _ = spotUtils_ros.getPosition(node.pose_state)
                    #####AGGIORNA LA POSIZIONE CON X,Y DI PRIMA NON LE RIPRENDE. DA VALUTARE QUALE SIA MEGLIO####
                    env.update_position(x_new, y_new)
                    env.print_map()
                    recording.create_default_waypoint(#### CONTROLLARE SE SERVONO DAVVERO QUEI PARAMETRI EXTRA
                        cell_row=selected_border[0],
                        cell_col=selected_border[1],
                        x=x_new,
                        y=y_new,
                        z=z_new,
                        yaw=node.pose_state.yaw(),
                    )
                    env.add_waypoint(x_new, y_new)
                    env.mark_cell_visited(selected_border[0], selected_border[1])
                    robot_row, robot_col = env.get_cell_from_world(x_new, y_new)
                    frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))
            else:
                lowest_rank_cell = env.get_lowest_rank_from_frontier_list(frontier, path)
                if lowest_rank_cell is None:### USA LA LOGICA IS NOT NONE PER ALLINEARTI  AL CODICE DI LA
                    break
                target_row, target_col, rank = lowest_rank_cell

                waypoints_by_cell = recording.get_all_manual_waypoints_with_cells()
                nearest_cell = recording.find_nearest_waypoint_cell_to_target((target_row, target_col), waypoints_by_cell, env)
                if nearest_cell is not None:
                    nearest_wp = recording.get_manual_waypoint_by_cell(nearest_cell[0], nearest_cell[1])
                    if nearest_wp is not None:
                        node.motion.move_to(nearest_wp['x'], nearest_wp['y'])
                        x_mid, y_mid, _, _ = spotUtils_ros.getPosition(node.pose_state)
                        env.add_robot_position(x_mid, y_mid, movement_type='navigate')

                ok = attempt_enter_cell_from_position(
                    node, env, target_row, target_col,
                    iteration=node.visualization_counter,
                    recordingInterface=recording,
                )
                node.visualization_counter += 1
                frontier.remove((target_row, target_col, rank))

                if ok:
                    x_final, y_final, z_final, _ = spotUtils_ros.getPosition(node.pose_state)
                    recording.create_default_waypoint(
                        cell_row=target_row,
                        cell_col=target_col,
                        x=x_final,
                        y=y_final,
                        z=z_final,
                        yaw=node.pose_state.yaw(),
                    )
                    env.add_waypoint(x_final, y_final)
                    env.mark_cell_visited(target_row, target_col)
                    robot_row, robot_col = env.get_cell_from_world(x_final, y_final)
                    frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))

            rclpy.spin_once(node, timeout_sec=0.1)

        node.get_logger().info('Exploration complete.')

        start_center = env.get_world_position_from_cell(*env.start_cell)
        if start_center is not None:
            node.motion.move_to(start_center[0], start_center[1])
        return True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    easy_walk()


if __name__ == '__main__':
    main()

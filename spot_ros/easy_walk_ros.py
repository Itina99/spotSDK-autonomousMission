#!/usr/bin/env python3
import os
import time
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener, TransformException

from spot_ros.environmentMap import EnvironmentMap
from spot_ros import movements_ros
from spot_ros import spotUtils_ros
from spot_ros import spotGrid_ros
from spot_ros import navGraphUtils_ros


def check_line_of_sight(
        x1,
        y1,
        x2,
        y2,
        grid_helper,
        occupied_threshold=65,
        max_occupied_ratio=0.25,
        min_consecutive_hits=3,
):
    """Verifica LOS conservativa su OccupancyGrid.

    Corrisponde a `easy_walk.py::check_line_of_sight`, adattata al backend ROS
    (`grid_helper` + soglia occupancy) invece della griglia `obstacle_distance`.
    """
    is_clear, _ = evaluate_line_of_sight(
        x1,
        y1,
        x2,
        y2,
        grid_helper,
        occupied_threshold=occupied_threshold,
        max_occupied_ratio=max_occupied_ratio,
        min_consecutive_hits=min_consecutive_hits,
    )
    return is_clear


def evaluate_line_of_sight(
        x1,
        y1,
        x2,
        y2,
        grid_helper,
        occupied_threshold=65,
        max_occupied_ratio=0.25,
        min_consecutive_hits=3,
):
    """Valuta LOS e restituisce anche metriche di diagnostica."""
    distance = math.hypot(x2 - x1, y2 - y1)
    if distance < 1e-6:
        return True, {
            'reason': 'degenerate_segment',
            'valid_checks': 0,
            'occupied_hits': 0,
            'occupied_ratio': 0.0,
        }

    num_checks = max(20, int(distance * 15.0))
    valid_checks = 0
    occupied_hits = 0
    consecutive_hits = 0

    for i in range(num_checks):
        t = i / max(1, num_checks - 1)
        check_x = x1 + t * (x2 - x1)
        check_y = y1 + t * (y2 - y1)

        map_idx = grid_helper.world_to_map(check_x, check_y)
        if map_idx is None:
            # Out-of-map samples are ignored to avoid pessimistic blocking near boundaries.
            continue

        mx, my = map_idx
        value = grid_helper.data[my * grid_helper.width + mx]
        if value < 0:
            # Unknown cells are ignored here; hard blocking them creates too many false negatives.
            continue

        valid_checks += 1
        if value >= occupied_threshold:
            occupied_hits += 1
            consecutive_hits += 1
            if consecutive_hits >= min_consecutive_hits:
                ratio = occupied_hits / float(max(1, valid_checks))
                return False, {
                    'reason': 'consecutive_hits',
                    'valid_checks': valid_checks,
                    'occupied_hits': occupied_hits,
                    'occupied_ratio': ratio,
                }
        else:
            consecutive_hits = 0

    if valid_checks == 0:
        return True, {
            'reason': 'no_valid_checks',
            'valid_checks': 0,
            'occupied_hits': 0,
            'occupied_ratio': 0.0,
        }

    ratio = occupied_hits / float(valid_checks)
    is_clear = ratio <= max_occupied_ratio
    return is_clear, {
        'reason': 'ok' if is_clear else 'occupied_ratio',
        'valid_checks': valid_checks,
        'occupied_hits': occupied_hits,
        'occupied_ratio': ratio,
    }


def sample_cell_points(env, cell_row, cell_col, num_samples=200):
    """Campiona punti casuali nella cella target in coordinate mondo.

    Corrisponde a `easy_walk.py::sample_cell_points`.
    """
    world_pos = env.get_world_position_from_cell(cell_row, cell_col)
    if world_pos is None:
        return []

    cell_center_x, cell_center_y = world_pos
    half_size = env.cell_size / 2.0

    samples = []
    for _ in range(num_samples):
        offset_x = (random_uniform(-half_size * 0.8, half_size * 0.8))
        offset_y = (random_uniform(-half_size * 0.8, half_size * 0.8))

        cos_yaw = math.cos(env.origin_yaw)
        sin_yaw = math.sin(env.origin_yaw)

        world_offset_x = offset_x * cos_yaw - offset_y * sin_yaw
        world_offset_y = offset_x * sin_yaw + offset_y * cos_yaw

        sample_x = cell_center_x + world_offset_x
        sample_y = cell_center_y + world_offset_y

        samples.append((sample_x, sample_y))

    return samples


def random_uniform(a, b):
    """Fallback RNG leggero senza dipendenza da NumPy.

    In `easy_walk.py::sample_cell_points` viene usato `np.random.uniform`.
    """
    return (b - a) * (os.urandom(4)[0] / 255.0) + a


def find_best_point_in_cell(
        robot_x,
        robot_y,
        env,
        cell_row,
        cell_col,
        grid_helper,
        occupied_threshold=65,
        max_occupied_ratio=0.25,
        min_consecutive_hits=3,
):
    """Seleziona il punto valido piu vicino al centro cella.

    Corrisponde a `easy_walk.py::find_best_point_in_cell`, con controllo LOS ROS.
    """
    sampled_points = sample_cell_points(env, cell_row, cell_col, num_samples=100)
    if not sampled_points:
        return None, None, [], [], {
            'total_samples': 0,
            'valid_samples': 0,
            'rejected_samples': 0,
            'reject_reasons': {},
            'avg_rejected_occupied_ratio': 0.0,
        }

    cell_center = env.get_world_position_from_cell(cell_row, cell_col)
    if cell_center is None:
        return None, None, [], [], {
            'total_samples': len(sampled_points),
            'valid_samples': 0,
            'rejected_samples': len(sampled_points),
            'reject_reasons': {'missing_cell_center': len(sampled_points)},
            'avg_rejected_occupied_ratio': 0.0,
        }

    cell_center_x, cell_center_y = cell_center

    valid_samples = []
    rejected_samples = []
    rejected_dump = []
    rejected_ratio_sum = 0.0
    reject_reasons = {}

    for sample_x, sample_y in sampled_points:
        is_clear, los_info = evaluate_line_of_sight(
            robot_x,
            robot_y,
            sample_x,
            sample_y,
            grid_helper,
            occupied_threshold=occupied_threshold,
            max_occupied_ratio=max_occupied_ratio,
            min_consecutive_hits=min_consecutive_hits,
        )
        if is_clear:
            valid_samples.append((sample_x, sample_y))
        else:
            rejected_samples.append((sample_x, sample_y))
            reason = str(los_info.get('reason', 'unknown'))
            reject_reasons[reason] = reject_reasons.get(reason, 0) + 1
            rejected_ratio_sum += float(los_info.get('occupied_ratio', 0.0))
            rejected_dump.append({
                'x': round(sample_x, 3),
                'y': round(sample_y, 3),
                'reason': reason,
                'occupied_ratio': round(float(los_info.get('occupied_ratio', 0.0)), 3),
                'occupied_hits': int(los_info.get('occupied_hits', 0)),
                'valid_checks': int(los_info.get('valid_checks', 0)),
            })

    los_stats = {
        'total_samples': len(sampled_points),
        'valid_samples': len(valid_samples),
        'rejected_samples': len(rejected_samples),
        'reject_reasons': reject_reasons,
        'rejected_dump': rejected_dump,
        'avg_rejected_occupied_ratio': (
            rejected_ratio_sum / float(len(rejected_samples))
            if rejected_samples else 0.0
        ),
    }

    if not valid_samples:
        return None, None, valid_samples, rejected_samples, los_stats

    best_point = None
    min_distance = float('inf')

    for sample_x, sample_y in valid_samples:
        dist = math.hypot(sample_x - cell_center_x, sample_y - cell_center_y)
        if dist < min_distance:
            min_distance = dist
            best_point = (sample_x, sample_y)

    los_stats['chosen_los_sample'] = {
        'x': round(best_point[0], 3),
        'y': round(best_point[1], 3),
        'distance_to_center': round(min_distance, 3),
    }

    return best_point[0], best_point[1], valid_samples, rejected_samples, los_stats


def visualize_grid_with_candidates_ros(node, grid_helper, env, robot_x, robot_y, candidates, chosen_point, iteration,
                                       save_path=None):
    """Pubblica in RViz la stessa diagnostica visuale di `easy_walk.py::visualize_grid_with_candidates`.

    Corrisponde alla versione Matplotlib, ma tramite `visualization_msgs/MarkerArray`.
    """
    if node is None or not hasattr(node, 'viz_pub'):
        return
    if not bool(node.get_parameter('viz_enabled').value):
        return

    frame_id = node.map_frame if node.map_frame else 'map'
    stamp = node.get_clock().now().to_msg()
    marker_array = MarkerArray()
    marker_id = 0

    def rgba(r, g, b, a=1.0):
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

    def pt(x, y, z=0.03):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def add_marker(ns, mtype, color, sx, sy, sz, action=Marker.ADD):
        nonlocal marker_id
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = marker_id
        marker_id += 1
        m.type = mtype
        m.action = action
        m.pose.orientation.w = 1.0
        m.scale.x = float(sx)
        m.scale.y = float(sy)
        m.scale.z = float(sz)
        m.color = color
        marker_array.markers.append(m)
        return m

    clear = add_marker('easy_walk_clear', Marker.SPHERE, rgba(0, 0, 0, 0), 0.01, 0.01, 0.01, action=Marker.DELETEALL)
    clear.id = 0
    marker_id = 1

    occupied_threshold = int(node.get_parameter('occupied_threshold').value)
    padding_threshold = int(node.get_parameter('viz_padding_threshold').value)
    max_points = max(1000, int(node.get_parameter('viz_max_points').value))
    local_radius = max(2.0, float(node.get_parameter('viz_local_radius').value))
    res = max(0.03, float(grid_helper.resolution))

    occupied_points = []
    padding_points = []
    free_points = []
    unknown_points = []

    stride = max(1, int(math.sqrt((grid_helper.width * grid_helper.height) / float(max_points))))
    for my in range(0, grid_helper.height, stride):
        for mx in range(0, grid_helper.width, stride):
            wx, wy = grid_helper.map_to_world(mx, my)
            if abs(wx - robot_x) > local_radius or abs(wy - robot_y) > local_radius:
                continue
            value = grid_helper.data[my * grid_helper.width + mx]
            point = pt(wx, wy, 0.02)
            if value < 0:
                unknown_points.append(point)
            elif value >= occupied_threshold:
                occupied_points.append(point)
            elif value >= padding_threshold:
                padding_points.append(point)
            else:
                free_points.append(point)

    grid_markers = [
        ('grid_occupied', occupied_points, rgba(1.0, 0.0, 0.0, 0.55)),
        ('grid_padding', padding_points, rgba(0.0, 1.0, 0.0, 0.50)),
        ('grid_free', free_points, rgba(0.0, 0.0, 1.0, 0.35)),
        ('grid_unknown', unknown_points, rgba(0.6, 0.6, 0.6, 0.22)),
    ]
    for ns, points, color in grid_markers:
        if not points:
            continue
        m = add_marker(ns, Marker.POINTS, color, res * 0.85, res * 0.85, 0.01)
        m.points = points

    if env is not None:
        cos_yaw = math.cos(env.origin_yaw)
        sin_yaw = math.sin(env.origin_yaw)
        half_size = env.cell_size / 2.0

        visited_lines = add_marker('cells_visited', Marker.LINE_LIST, rgba(0.0, 0.5, 0.0, 0.85), 0.05, 0.0, 0.0)
        blocked_lines = add_marker('cells_blocked', Marker.LINE_LIST, rgba(0.7, 0.0, 0.0, 0.85), 0.05, 0.0, 0.0)
        unvisited_lines = add_marker('cells_unvisited', Marker.LINE_LIST, rgba(0.5, 0.5, 0.5, 0.55), 0.03, 0.0, 0.0)

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
                    cell_status = status_raw[0]
                else:
                    cell_status = status_raw

                target_lines = unvisited_lines
                if cell_status == 1:
                    target_lines = visited_lines
                elif cell_status == -1:
                    target_lines = blocked_lines

                for i in range(4):
                    target_lines.points.append(corners[i])
                    target_lines.points.append(corners[(i + 1) % 4])

                t = add_marker('cell_labels', Marker.TEXT_VIEW_FACING, rgba(0.0, 0.0, 0.0, 0.85), 0.0, 0.0, 0.18)
                t.pose.position.x = float(cell_x)
                t.pose.position.y = float(cell_y)
                t.pose.position.z = 0.08
                t.text = f"{row},{col}"

    rejected_los = []
    rejected_corridor = []
    valid_los = []
    valid_corridor = []
    if isinstance(candidates, dict):
        # Backward compatible keys:
        # - rejected / valid (legacy)
        # - rejected_los / valid_los / rejected_corridor / valid_corridor (new)
        rejected_los = list(candidates.get('rejected_los', candidates.get('rejected', [])))
        rejected_corridor = list(candidates.get('rejected_corridor', []))
        valid_los = list(candidates.get('valid_los', candidates.get('valid', [])))
        valid_corridor = list(candidates.get('valid_corridor', []))

    if rejected_los:
        m = add_marker('candidates_rejected_los', Marker.POINTS, rgba(1.0, 0.0, 0.0, 1.0), 0.14, 0.14, 0.01)
        m.points = [pt(px, py, 0.06) for px, py in rejected_los]

    if rejected_corridor:
        m = add_marker('candidates_rejected_corridor', Marker.POINTS, rgba(1.0, 0.55, 0.0, 1.0), 0.13, 0.13, 0.01)
        m.points = [pt(px, py, 0.07) for px, py in rejected_corridor]

    if valid_los:
        m = add_marker('candidates_valid_los', Marker.POINTS, rgba(1.0, 0.92, 0.0, 0.55), 0.12, 0.12, 0.01)
        m.points = [pt(px, py, 0.055) for px, py in valid_los]

    if valid_corridor:
        m = add_marker('candidates_valid_corridor', Marker.POINTS, rgba(0.1, 1.0, 1.0, 0.95), 0.15, 0.15, 0.01)
        m.points = [pt(px, py, 0.08) for px, py in valid_corridor]

    robot_marker = add_marker('robot_pose', Marker.SPHERE, rgba(0.0, 0.1, 1.0, 0.95), 0.28, 0.28, 0.16)
    robot_marker.pose.position = pt(robot_x, robot_y, 0.10)

    for radius, ns in ((1.0, 'robot_range_1m'), (2.0, 'robot_range_2m')):
        c = add_marker(ns, Marker.LINE_STRIP, rgba(0.0, 0.2, 1.0, 0.45), 0.02, 0.0, 0.0)
        steps = 48
        for i in range(steps + 1):
            a = (2.0 * math.pi * i) / steps
            c.points.append(pt(robot_x + radius * math.cos(a), robot_y + radius * math.sin(a), 0.03))

    if chosen_point is not None:
        tx, ty = chosen_point
        target = add_marker('chosen_target', Marker.SPHERE, rgba(0.0, 0.9, 0.0, 1.0), 0.30, 0.30, 0.20)
        target.pose.position = pt(tx, ty, 0.10)

        target_line = add_marker('target_segment', Marker.LINE_STRIP, rgba(0.0, 0.8, 0.0, 0.95), 0.05, 0.0, 0.0)
        target_line.points = [pt(robot_x, robot_y, 0.05), pt(tx, ty, 0.05)]

        dist = math.hypot(tx - robot_x, ty - robot_y)
        label = add_marker('target_distance', Marker.TEXT_VIEW_FACING, rgba(0.0, 0.4, 0.0, 1.0), 0.0, 0.0, 0.22)
        label.pose.position = pt((robot_x + tx) / 2.0, (robot_y + ty) / 2.0, 0.22)
        label.text = f"{dist:.2f}m"

    if env is not None and hasattr(env, 'waypoints') and isinstance(env.waypoints, list) and len(env.waypoints) > 0:
        wp_points = add_marker('waypoints', Marker.POINTS, rgba(1.0, 0.0, 1.0, 0.95), 0.16, 0.16, 0.01)
        for i, wp in enumerate(env.waypoints):
            if not isinstance(wp, (tuple, list)) or len(wp) < 2:
                continue
            wp_x = float(wp[0])
            wp_y = float(wp[1])
            wp_points.points.append(pt(wp_x, wp_y, 0.08))

            txt = add_marker('waypoint_labels', Marker.TEXT_VIEW_FACING, rgba(0.4, 0.0, 0.5, 1.0), 0.0, 0.0, 0.20)
            txt.pose.position = pt(wp_x + 0.12, wp_y + 0.12, 0.20)
            txt.text = f"W{i + 1}"

    if env is not None and hasattr(env, 'robot_path') and isinstance(env.robot_path, list) and len(env.robot_path) > 0:
        nav_lines = add_marker('path_navigate', Marker.LINE_LIST, rgba(1.0, 0.2, 0.0, 0.8), 0.04, 0.0, 0.0)
        exp_lines = add_marker('path_explore', Marker.LINE_LIST, rgba(0.0, 0.9, 0.2, 0.8), 0.04, 0.0, 0.0)
        nav_pts = add_marker('path_points_navigate', Marker.POINTS, rgba(1.0, 0.6, 0.0, 0.9), 0.06, 0.06, 0.01)
        exp_pts = add_marker('path_points_explore', Marker.POINTS, rgba(0.3, 1.0, 0.2, 0.9), 0.06, 0.06, 0.01)

        parsed = []
        for entry in env.robot_path:
            if not isinstance(entry, (tuple, list)) or len(entry) < 2:
                continue
            movement_type = str(entry[2]) if len(entry) >= 3 else 'explore'
            parsed.append((float(entry[0]), float(entry[1]), movement_type))

        for px, py, movement_type in parsed:
            if movement_type == 'navigate':
                nav_pts.points.append(pt(px, py, 0.07))
            else:
                exp_pts.points.append(pt(px, py, 0.07))

        for i in range(len(parsed) - 1):
            p1 = parsed[i]
            p2 = parsed[i + 1]
            segment = [pt(p1[0], p1[1], 0.05), pt(p2[0], p2[1], 0.05)]
            if p1[2] == 'navigate' or p2[2] == 'navigate':
                nav_lines.points.extend(segment)
            else:
                exp_lines.points.extend(segment)

    info = add_marker('iteration_info', Marker.TEXT_VIEW_FACING, rgba(0.1, 0.1, 0.1, 0.95), 0.0, 0.0, 0.24)
    info.pose.position = pt(robot_x, robot_y + 0.45, 0.28)
    info.text = (
        f"iter={iteration} valid_los={len(valid_los)} valid_corridor={len(valid_corridor)} "
        f"rej_los={len(rejected_los)} rej_corridor={len(rejected_corridor)}"
    )

    node.viz_pub.publish(marker_array)


class EasyWalkROS(Node):
    def __init__(self):
        """Inizializza nodo ROS, parametri, subscriber/publisher e controller di moto.

        Corrisponde al bootstrap di `easy_walk.py::easy_walk` (inizializzazione missione),
        ma nel modello ROS event-driven.
        """
        super().__init__('easy_walk_ros')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('grid_rows', 4)
        self.declare_parameter('grid_cols', 4)
        self.declare_parameter('cell_size', 2.0)
        self.declare_parameter('control_frame', 'odom')
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
        self.declare_parameter('waypoint_center_tolerance', 0.05)
        self.declare_parameter('moving_angular_scale', 0.4)
        self.declare_parameter('moving_angular_cap', 0.25)
        self.declare_parameter('frontier_verbose', False)
        self.declare_parameter('frontier_decision_logs_enabled', True)
        self.declare_parameter('sample_decision_logs_enabled', True)
        self.declare_parameter('sample_rejected_dump_limit', 8)
        self.declare_parameter('sample_corridor_dump_limit', 8)
        self.declare_parameter('debug_attempts', True)
        self.declare_parameter('sdk_like_discard_adjacent_on_fail', True)
        self.declare_parameter('sdk_like_skip_retreat_on_adjacent_fail', True)
        self.declare_parameter('max_transient_retries', 2)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('max_occupied_ratio', 0.25)
        self.declare_parameter('min_consecutive_hits', 3)
        self.declare_parameter('avoid_enabled', True)
        self.declare_parameter('avoid_step_size', 0.8)
        self.declare_parameter('avoid_lateral_offset', 0.6)
        self.declare_parameter('avoid_max_retries', 2)
        self.declare_parameter('avoid_sample_step', 0.1)
        self.declare_parameter('avoid_unknown_as_obstacle', False)
        self.declare_parameter('avoid_occupied_threshold', 45)
        self.declare_parameter('avoid_partial_threshold', 25)
        self.declare_parameter('avoid_corridor_half_width', 0.22)
        self.declare_parameter('avoid_retreat_steps', 2)
        self.declare_parameter('avoid_max_steps', 250)
        self.declare_parameter('avoid_stall_window', 8)
        self.declare_parameter('avoid_progress_epsilon', 0.01)
        self.declare_parameter('avoid_goal_hysteresis', 0.03)
        self.declare_parameter('local_obs_enabled', True)
        self.declare_parameter('local_obs_scan_topic', '/spot/lidar/scan')
        self.declare_parameter('lidar_waypoint_scan_enabled', True)
        self.declare_parameter('lidar_waypoint_scan_window_sec', 0.7)
        self.declare_parameter('lidar_burst_only_mode', True)
        self.declare_parameter('use_lidar_for_navigation', False)
        self.declare_parameter('local_obs_timeout', 0.8)
        self.declare_parameter('local_obs_min_range', 0.12)
        self.declare_parameter('local_obs_max_range', 3.5)
        self.declare_parameter('local_obs_stride', 2)
        self.declare_parameter('local_obs_hit_radius', 0.20)
        self.declare_parameter('camera_local_grid_enabled', True)
        self.declare_parameter('camera_local_grid_topic', '/spot/camera/local_obstacles')
        self.declare_parameter('camera_local_grid_timeout', 0.6)
        self.declare_parameter('camera_local_grid_occupied_threshold', 30)
        self.declare_parameter('viz_enabled', True)
        self.declare_parameter('viz_topic', '/easy_walk/visualization')
        self.declare_parameter('viz_max_points', 9000)
        self.declare_parameter('viz_local_radius', 8.0)
        self.declare_parameter('viz_padding_threshold', 15)

        map_topic = self.get_parameter('map_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        scan_topic = self.get_parameter('local_obs_scan_topic').value
        camera_local_grid_topic = self.get_parameter('camera_local_grid_topic').value
        viz_topic = self.get_parameter('viz_topic').value

        self.pose_state = spotUtils_ros.PoseState()
        self.pose_state_odom = spotUtils_ros.PoseState()
        self.last_odom_stamp = None
        self.map_frame = None
        self.odom_frame = None
        self._tf_warned = False
        self.current_map = None
        self._map_debug_printed = False
        self._local_obs_points_map = []
        self._local_obs_stamp_sec = None
        self._camera_local_grid_helper = None
        self._camera_local_grid_stamp_sec = None
        self._accept_lidar_scans = False
        self._lidar_burst_active = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.viz_pub = self.create_publisher(MarkerArray, viz_topic, 1)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 1)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_subscription(OccupancyGrid, camera_local_grid_topic, self._on_camera_local_grid, 1)

        self.motion = movements_ros.MotionController(
            get_pose_fn=self._get_control_pose,
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

        mission_timestamp = datetime.now().strftime("Mission_%d-%m-%Y_%H-%M-%S")
        self.mission_folder = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'MissionMap',
            mission_timestamp,
        )
        os.makedirs(self.mission_folder, exist_ok=True)
        self.visualization_counter = 0
        self._cell_retry_count = {}

    @staticmethod
    def _cell_key(row, col):
        """Normalizza identificativo cella per il tracking dei retry."""
        return f"{row},{col}"

    def _is_transient_failure(self, reason):
        """Classifica errori recuperabili prima di marcare la cella come bloccata."""
        return reason in {
            'ROTATE_TIMEOUT',
            'MOVE_TIMEOUT',
            'MOVE_NO_PROGRESS',
            'TF_UNAVAILABLE',
            'AVOIDANCE_BLOCKED',
            'AVOIDANCE_STALLED',
            'AVOIDANCE_MAX_STEPS',
        }

    def _transform_point_2d(self, x, y, from_frame, to_frame):
        """Trasforma un punto 2D tra frame TF.

        Non ha equivalente diretto in `easy_walk.py` (lato SDK usa frame VISION tramite API Bosdyn).
        """
        if from_frame == to_frame:
            return x, y

        stamp = self.last_odom_stamp if self.last_odom_stamp is not None else Time()
        try:
            tf = self.tf_buffer.lookup_transform(to_frame, from_frame, stamp)
        except TransformException:
            # Fallback to latest transform when exact timestamp is unavailable.
            tf = self.tf_buffer.lookup_transform(to_frame, from_frame, Time())
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        rq = tf.transform.rotation
        tf_yaw = self._quat_to_yaw(rq.x, rq.y, rq.z, rq.w)

        c = math.cos(tf_yaw)
        s = math.sin(tf_yaw)
        x_out = tx + c * x - s * y
        y_out = ty + s * x + c * y
        return x_out, y_out

    def _handle_failed_attempt(self, env, frontier, row, col, rank, reason, details, attempt_kind='fallback'):
        """Gestisce fallimenti di ingresso cella con retry/backoff e blocco permanente.

        Corrisponde alla logica di fallback distribuita in `easy_walk.py::easy_walk`
        (rimozione da frontier/skip), con diagnostica piu strutturata.
        """
        key = self._cell_key(row, col)
        debug_attempts = bool(self.get_parameter('debug_attempts').value)

        retreat_reasons = {'MOVE_NO_PROGRESS', 'AVOIDANCE_BLOCKED', 'AVOIDANCE_STALLED'}
        details_payload = dict(details) if isinstance(details, dict) else {'raw_details': str(details)}
        sdk_like_discard_adjacent = bool(self.get_parameter('sdk_like_discard_adjacent_on_fail').value)
        sdk_like_skip_retreat = bool(self.get_parameter('sdk_like_skip_retreat_on_adjacent_fail').value)

        if attempt_kind == 'adjacent' and sdk_like_discard_adjacent:
            self._cell_retry_count[key] = 0
            self._log_frontier_decision(
                f"discard adjacent cell=({row},{col}) reason={reason} details={details_payload}"
            )
            if debug_attempts:
                self.get_logger().debug(
                    f"[SDK_PARITY] adjacent_fail_discarded cell=({row},{col}) rank={rank} reason={reason}"
                )
            return frontier

        if reason in retreat_reasons and not (attempt_kind == 'adjacent' and sdk_like_skip_retreat):
            retreat_steps = max(1, int(self.get_parameter('avoid_retreat_steps').value))
            retreat_ok = self._retreat_to_previous_waypoint(env, retreat_steps=retreat_steps)
            details_payload['retreat_attempted'] = True
            details_payload['retreat_ok'] = retreat_ok

        if self._is_transient_failure(reason):
            self._cell_retry_count[key] = self._cell_retry_count.get(key, 0) + 1
            retries = self._cell_retry_count[key]
            max_retries = int(self.get_parameter('max_transient_retries').value)

            if retries <= max_retries:
                if not any((f[0] == row and f[1] == col) for f in frontier):
                    frontier.append((row, col, rank))
                if debug_attempts:
                    self.get_logger().warn(
                        f"[ATTEMPT_FAIL] cell=({row},{col}) transient={reason} retry={retries}/{max_retries} details={details_payload}"
                    )
            else:
                if debug_attempts:
                    self.get_logger().warn(
                        f"[ATTEMPT_FAIL] cell=({row},{col}) transient={reason} retries_exhausted={retries}/{max_retries} details={details_payload}. Not marking blocked."
                    )
            return frontier

        self._cell_retry_count[key] = 0
        env.mark_cell_blocked(row, col)
        self.get_logger().warn(
            f"[ATTEMPT_FAIL] cell=({row},{col}) permanent={reason} -> marked_blocked details={details_payload}"
        )
        return frontier

    @staticmethod
    def _sort_frontier(frontier):
        """Ordina la frontier per rank e coordinate.

        Corrisponde alla selezione `min(..., key=lambda b: b[2])` in `easy_walk.py::easy_walk`.
        """
        return sorted(frontier, key=lambda c: (c[2], c[0], c[1]))

    @staticmethod
    def _format_cells(cells, limit=8):
        """Compatta una lista celle per log diagnostici."""
        if not cells:
            return '[]'
        preview = cells[:limit]
        txt = ', '.join([f"({r},{c},rank={rank})" for r, c, rank in preview])
        if len(cells) > limit:
            txt += f", ... +{len(cells) - limit}"
        return f"[{txt}]"

    def _log_frontier_decision(self, msg):
        """Canale diagnostico dedicato alla scelta frontiera."""
        if bool(self.get_parameter('frontier_decision_logs_enabled').value):
            self.get_logger().warn(f"[FRONTIER_DECISION] {msg}")

    def _log_sample_decision(self, msg):
        """Canale diagnostico dedicato alla selezione/scarto sample in attempt_enter_cell."""
        if bool(self.get_parameter('sample_decision_logs_enabled').value):
            self.get_logger().warn(f"[SAMPLE_DECISION] {msg}")

    def _log_frontier(self, frontier, label):
        """Logga lo stato della frontier in formato stabile (opzionale)."""
        if not bool(self.get_parameter('frontier_verbose').value):
            return
        ordered = self._sort_frontier(frontier)
        if not ordered:
            self.get_logger().info(f"[FRONTIER] {label}: empty")
            return
        cells = ', '.join([f"({r},{c},rank={rank})" for r, c, rank in ordered])
        self.get_logger().info(f"[FRONTIER] {label}: {len(ordered)} cells -> [{cells}]")

    def _merge_frontier(self, frontier, candidates, reason):
        """Aggiunge celle candidate evitando duplicati.

        Corrisponde a `easy_walk.py::find_new_borders` + `frontier.extend(...)`.
        """
        existing = {(r, c) for r, c, _ in frontier}
        added = []
        for r, c, rank in candidates:
            if (r, c) in existing:
                continue
            frontier.append((r, c, rank))
            existing.add((r, c))
            added.append((r, c, rank))

        if added:
            self._log_frontier(added, f"added ({reason})")
        else:
            if bool(self.get_parameter('frontier_verbose').value):
                self.get_logger().info(f"[FRONTIER] added ({reason}): none")

        self._log_frontier(frontier, f"state after add ({reason})")
        return frontier

    def _remove_frontier_cell(self, frontier, row, col, reason):
        """Rimuove una cella dalla frontier dopo un tentativo (successo/fallimento).

        Corrisponde a `frontier.remove(...)` in `easy_walk.py::easy_walk`.
        """
        before = len(frontier)
        frontier = [f for f in frontier if not (f[0] == row and f[1] == col)]
        removed = before - len(frontier)
        if bool(self.get_parameter('frontier_verbose').value):
            self.get_logger().info(
                f"[FRONTIER] removed ({reason}): cell=({row},{col}) removed_count={removed}"
            )
        self._log_frontier(frontier, f"state after remove ({reason})")
        return frontier

    def _return_to_start_via_robot_path(self, env):
        """Rientra alla base ripercorrendo al contrario i punti esplorati.

        Corrisponde all'intento del blocco finale di ritorno in `easy_walk.py::easy_walk`
        (navigazione verso wp_0), ma qui usa il path locale registrato in `env.robot_path`.
        """
        if not hasattr(env, 'robot_path') or not isinstance(env.robot_path, list) or len(env.robot_path) < 2:
            self.get_logger().warn('[RETURN] Robot path not available; skipping return path execution.')
            return False

        points = []
        for entry in env.robot_path:
            if isinstance(entry, (tuple, list)) and len(entry) >= 2:
                points.append((float(entry[0]), float(entry[1])))

        if len(points) < 2:
            self.get_logger().warn('[RETURN] Not enough path points to backtrack.')
            return False

        targets = list(reversed(points[:-1]))
        self.get_logger().info(f"[RETURN] Backtracking on {len(targets)} path points.")

        for idx, (tx, ty) in enumerate(targets, start=1):
            target_x = tx
            target_y = ty
            control_frame = str(self.get_parameter('control_frame').value).lower().strip()
            if control_frame == 'odom' and self.map_frame is not None and self.odom_frame is not None:
                try:
                    target_x, target_y = self._transform_point_2d(tx, ty, self.map_frame, self.odom_frame)
                except TransformException as exc:
                    self.get_logger().warn(f"[RETURN] TF unavailable for map->odom conversion: {exc}")
                    return False

            ok = self.motion.move_to(target_x, target_y, timeout=45.0)
            if not ok:
                self.get_logger().warn(
                    f"[RETURN] Failed at step {idx}/{len(targets)} towards ({target_x:.3f},{target_y:.3f})."
                )
                return False
            x_now, y_now, _, _ = spotUtils_ros.getPosition(self.pose_state)
            env.add_robot_position(x_now, y_now, movement_type='navigate')

        self.get_logger().info('[RETURN] Robot returned to start by backtracking.')
        return True

    def _retreat_to_previous_waypoint(self, env, retreat_steps=2):
        """Fallback locale: torna a un waypoint precedente dopo uno stallo.

        Non ha equivalente diretto in `easy_walk.py`; e una protezione aggiunta per ROS.
        """
        if not hasattr(env, 'waypoints') or not isinstance(env.waypoints, list) or len(env.waypoints) < 2:
            self.get_logger().warn('[RETREAT] Not enough waypoints to retreat.')
            return False

        depth = max(1, int(retreat_steps))
        target_idx = max(0, len(env.waypoints) - 1 - depth)
        target = env.waypoints[target_idx]
        if not isinstance(target, (tuple, list)) or len(target) < 2:
            self.get_logger().warn('[RETREAT] Invalid waypoint format, aborting retreat.')
            return False

        tx = float(target[0])
        ty = float(target[1])
        self.get_logger().warn(
            f"[RETREAT] Moving to previous safe waypoint idx={target_idx} at ({tx:.3f},{ty:.3f})"
        )
        ok, reason, details = self._move_to_world_target(tx, ty)
        if not ok:
            self.get_logger().warn(f"[RETREAT] Failed reason={reason} details={details}")
            return False

        x_now, y_now, _, _ = spotUtils_ros.getPosition(self.pose_state)
        env.add_robot_position(x_now, y_now, movement_type='navigate')
        self.get_logger().info('[RETREAT] Completed.')
        return True

    def _log_alignment_debug(self, env, label, x, y, target_row=None, target_col=None):
        """Diagnostica allineamento posa->cella prima/dopo i movimenti."""
        if not bool(self.get_parameter('debug_attempts').value):
            return
        row, col = env.get_cell_from_world(x, y)
        msg = (
            f"[{label}] robot=({x:.3f},{y:.3f}) -> cell=({row},{col}) "
            f"map={env.rows}x{env.cols} cell_size={env.cell_size:.3f}"
        )
        if target_row is not None and target_col is not None:
            center = env.get_world_position_from_cell(target_row, target_col)
            if center is not None:
                err = math.hypot(x - center[0], y - center[1])
                msg += (
                    f" target=({target_row},{target_col}) center=({center[0]:.3f},{center[1]:.3f}) "
                    f"center_err={err:.3f}m"
                )
        self.get_logger().info(msg)

    def _is_segment_clear(self, grid_helper, x1, y1, x2, y2, occupied_threshold, treat_unknown_as_obstacle):
        """Controllo collisione su segmento campionato.

        Estensione ROS della verifica LOS di `easy_walk.py::check_line_of_sight`.
        """
        distance = math.hypot(x2 - x1, y2 - y1)
        if distance < 1e-6:
            return True

        sample_step = max(0.03, float(self.get_parameter('avoid_sample_step').value))
        samples = max(3, int(math.ceil(distance / sample_step)))
        partial_threshold = int(self.get_parameter('avoid_partial_threshold').value)

        for i in range(1, samples + 1):
            t = i / float(samples)
            sx = x1 + (x2 - x1) * t
            sy = y1 + (y2 - y1) * t
            map_idx = grid_helper.world_to_map(sx, sy)
            blocked_global = False
            if map_idx is None:
                blocked_global = bool(treat_unknown_as_obstacle)
            else:
                mx, my = map_idx
                value = grid_helper.data[my * grid_helper.width + mx]
                if value < 0:
                    blocked_global = bool(treat_unknown_as_obstacle)
                else:
                    blocked_global = value >= occupied_threshold or value >= partial_threshold

            blocked_local = self._is_local_obstacle_near(sx, sy)
            if blocked_global or blocked_local:
                if bool(self.get_parameter('debug_attempts').value):
                    reason = 'local_obs' if blocked_local else 'global_map'
                    self.get_logger().info(
                        f"[AVOID_DBG] segment_blocked sample={i}/{samples} at ({sx:.3f},{sy:.3f}) "
                        f"from=({x1:.3f},{y1:.3f}) to=({x2:.3f},{y2:.3f}) reason={reason}"
                    )
                return False
        return True

    def _has_fresh_lidar_observation(self):
        if not bool(self.get_parameter('local_obs_enabled').value):
            return False
        if bool(self.get_parameter('lidar_burst_only_mode').value):
            # In burst-only mode the lidar callback is used only to acquire snapshots at waypoints,
            # never as a continuous local obstacle source for in-motion navigation.
            return False
        if not bool(self.get_parameter('use_lidar_for_navigation').value):
            return False
        if self._local_obs_stamp_sec is None:
            return False
        timeout = max(0.05, float(self.get_parameter('local_obs_timeout').value))
        return (self._now_seconds() - self._local_obs_stamp_sec) <= timeout

    def _has_fresh_local_observation(self):
        # Camera local grid is the default local source; lidar is optional and usually disabled in-motion.
        return self._has_fresh_camera_local_grid() or self._has_fresh_lidar_observation()

    def _is_local_obstacle_near(self, x, y):
        if self._has_fresh_lidar_observation():
            hit_radius = max(0.05, float(self.get_parameter('local_obs_hit_radius').value))
            hit_radius_sq = hit_radius * hit_radius
            for ox, oy in self._local_obs_points_map:
                dx = x - ox
                dy = y - oy
                if (dx * dx + dy * dy) <= hit_radius_sq:
                    return True

        if self._has_fresh_camera_local_grid() and self._camera_local_grid_helper is not None:
            idx = self._camera_local_grid_helper.world_to_map(x, y)
            if idx is not None:
                mx, my = idx
                value = self._camera_local_grid_helper.data[my * self._camera_local_grid_helper.width + mx]
                if value >= int(self.get_parameter('camera_local_grid_occupied_threshold').value):
                    return True

        return False

    def _has_fresh_camera_local_grid(self):
        if not bool(self.get_parameter('camera_local_grid_enabled').value):
            return False
        if self._camera_local_grid_stamp_sec is None:
            return False
        timeout = max(0.05, float(self.get_parameter('camera_local_grid_timeout').value))
        return (self._now_seconds() - self._camera_local_grid_stamp_sec) <= timeout

    def _capture_lidar_snapshot_at_waypoint(self, label='waypoint'):
        """Acquire a short lidar burst while stationary at a waypoint (disabled during motion)."""
        if not bool(self.get_parameter('local_obs_enabled').value):
            return
        if not bool(self.get_parameter('lidar_waypoint_scan_enabled').value):
            return

        window = max(0.05, float(self.get_parameter('lidar_waypoint_scan_window_sec').value))
        self.get_logger().info(f"[LIDAR_BURST] trigger label={label} window={window:.2f}s")
        previous_accept = self._accept_lidar_scans
        previous_burst_state = self._lidar_burst_active
        stamp_before = self._local_obs_stamp_sec
        self._accept_lidar_scans = True
        self._lidar_burst_active = True
        try:
            deadline = self._now_seconds() + window
            while rclpy.ok() and self._now_seconds() < deadline:
                remaining = max(0.0, deadline - self._now_seconds())
                rclpy.spin_once(self, timeout_sec=min(0.05, remaining))
        finally:
            self._accept_lidar_scans = previous_accept
            self._lidar_burst_active = previous_burst_state

        if bool(self.get_parameter('debug_attempts').value):
            got_fresh = (
                self._local_obs_stamp_sec is not None and
                (stamp_before is None or self._local_obs_stamp_sec > stamp_before)
            )
            self.get_logger().info(
                f"[LIDAR_WAYPOINT] label={label} window={window:.2f}s fresh_scan={got_fresh}"
            )

    def _reach_waypoint_center_for_cell(self, env, target_row, target_col):
        """Raggiunge il centro cella (waypoint) prima di fare il burst lidar."""
        center = env.get_world_position_from_cell(target_row, target_col)
        if center is None:
            return False, 'NO_CELL_CENTER', {'target_row': target_row, 'target_col': target_col}

        center_x, center_y = center
        x_now, y_now, _, _ = spotUtils_ros.getPosition(self.pose_state)
        center_err = math.hypot(center_x - x_now, center_y - y_now)
        center_tol = max(0.02, float(self.get_parameter('waypoint_center_tolerance').value))

        if center_err <= center_tol:
            return True, 'ALREADY_AT_CENTER', {
                'center_x': round(center_x, 3),
                'center_y': round(center_y, 3),
                'center_err': round(center_err, 3),
            }

        ok, reason, details = self._move_to_world_target(center_x, center_y)
        if not ok:
            payload = {
                'center_x': round(center_x, 3),
                'center_y': round(center_y, 3),
                'center_err': round(center_err, 3),
            }
            if isinstance(details, dict):
                payload.update(details)
            return False, reason, payload

        x_after, y_after, _, _ = spotUtils_ros.getPosition(self.pose_state)
        env.add_robot_position(x_after, y_after, movement_type='explore')
        return True, 'OK', {
            'center_x': round(center_x, 3),
            'center_y': round(center_y, 3),
            'center_err': round(math.hypot(center_x - x_after, center_y - y_after), 3),
        }

    def _finalize_successful_cell_visit(self, env, recording, row, col):
        """Finalizza una cella visitata: centro waypoint, registrazione waypoint, burst lidar."""
        env.mark_cell_visited(row, col)

        reached_center, center_reason, center_details = self._reach_waypoint_center_for_cell(env, row, col)

        x_new, y_new, z_new, _ = spotUtils_ros.getPosition(self.pose_state)
        recording.create_default_waypoint(
            cell_row=row,
            cell_col=col,
            x=x_new,
            y=y_new,
            z=z_new,
            yaw=self.pose_state.yaw(),
        )
        env.add_waypoint(x_new, y_new)

        if reached_center:
            self._capture_lidar_snapshot_at_waypoint(label=f"cell_{row}_{col}")
        else:
            self.get_logger().warn(
                f"[LIDAR_WAYPOINT] skip burst cell=({row},{col}) "
                f"reason={center_reason} details={center_details}"
            )
        return x_new, y_new

    def _is_corridor_clear(self, grid_helper, x1, y1, x2, y2, occupied_threshold, treat_unknown_as_obstacle):
        """Verifica un corridoio (linea centrale + offset laterali).

        Non ha equivalente diretto in `easy_walk.py`; serve all'evitamento ostacoli step-by-step.
        """
        half_width = max(0.0, float(self.get_parameter('avoid_corridor_half_width').value))
        if half_width < 1e-3:
            return self._is_segment_clear(
                grid_helper,
                x1,
                y1,
                x2,
                y2,
                occupied_threshold,
                treat_unknown_as_obstacle,
            )

        distance = math.hypot(x2 - x1, y2 - y1)
        if distance < 1e-6:
            return True

        perp_x = -(y2 - y1) / distance
        perp_y = (x2 - x1) / distance
        offsets = (0.0, half_width, -half_width)
        for off in offsets:
            sx1 = x1 + perp_x * off
            sy1 = y1 + perp_y * off
            sx2 = x2 + perp_x * off
            sy2 = y2 + perp_y * off
            if not self._is_segment_clear(
                    grid_helper,
                    sx1,
                    sy1,
                    sx2,
                    sy2,
                    occupied_threshold,
                    treat_unknown_as_obstacle,
            ):
                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().info(
                        f"[AVOID_DBG] corridor_blocked offset={off:.2f} from=({x1:.3f},{y1:.3f}) to=({x2:.3f},{y2:.3f})"
                    )
                return False
        return True

    def _move_to_world_target(self, target_x, target_y):
        """Invia il movimento verso target mondo, con conversione map->odom se richiesta.

        Corrisponde alla fase di movimento in `easy_walk.py::attempt_enter_cell_from_position`
        (`movements.relative_move`), con controllo frame TF lato ROS.
        """
        control_frame = str(self.get_parameter('control_frame').value).lower().strip()
        use_odom_control = (control_frame == 'odom')

        if bool(self.get_parameter('debug_attempts').value):
            self.get_logger().info(
                f"[AVOID_DBG] move_request frame={control_frame} target_map=({target_x:.3f},{target_y:.3f})"
            )

        if use_odom_control:
            if self.map_frame is None or self.odom_frame is None:
                return False, 'TF_UNAVAILABLE', {
                    'reason': 'map_or_odom_frame_missing',
                    'map_frame': self.map_frame,
                    'odom_frame': self.odom_frame,
                }
            try:
                target_ctrl_x, target_ctrl_y = self._transform_point_2d(
                    target_x,
                    target_y,
                    self.map_frame,
                    self.odom_frame,
                )

                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().info(
                        f"[AVOID_DBG] map_to_odom target_odom=({target_ctrl_x:.3f},{target_ctrl_y:.3f}) "
                        f"map_frame={self.map_frame} odom_frame={self.odom_frame}"
                    )

                chk_map_x, chk_map_y = self._transform_point_2d(
                    target_ctrl_x,
                    target_ctrl_y,
                    self.odom_frame,
                    self.map_frame,
                )
                rt_err = math.hypot(chk_map_x - target_x, chk_map_y - target_y)
                if rt_err > 0.25:
                    if bool(self.get_parameter('debug_attempts').value):
                        self.get_logger().debug(
                            f"[AVOID_DBG] tf_roundtrip_failed rt_err={rt_err:.3f}m "
                            f"target_map=({target_x:.3f},{target_y:.3f})"
                        )
                    return False, 'TF_UNAVAILABLE', {
                        'reason': 'roundtrip_error_too_high',
                        'roundtrip_error': round(rt_err, 3),
                        'target_map': (round(target_x, 3), round(target_y, 3)),
                        'target_odom': (round(target_ctrl_x, 3), round(target_ctrl_y, 3)),
                    }
            except TransformException as exc:
                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().debug(f"[AVOID_DBG] transform_exception: {exc}")
                return False, 'TF_UNAVAILABLE', {
                    'reason': 'lookup_transform_failed',
                    'error': str(exc),
                    'from_frame': self.map_frame,
                    'to_frame': self.odom_frame,
                }
            ok_move = self.motion.move_to(target_ctrl_x, target_ctrl_y)
        else:
            ok_move = self.motion.move_to(target_x, target_y)

        if not ok_move:
            if bool(self.get_parameter('debug_attempts').value):
                self.get_logger().debug(
                    f"[AVOID_DBG] move_failed reason={self.motion.last_failure_reason} "
                    f"details={self.motion.last_failure_details}"
                )
            return False, self.motion.last_failure_reason, dict(self.motion.last_failure_details)

        if bool(self.get_parameter('debug_attempts').value):
            self.get_logger().info("[AVOID_DBG] move_ok")
        return True, 'OK', {}

    def _distance_to_target_in_control_frame(self, target_map_x, target_map_y):
        """Calcola distanza target nel frame di controllo effettivo (map/odom)."""
        control_frame = str(self.get_parameter('control_frame').value).lower().strip()
        if control_frame != 'odom':
            robot_x, robot_y, _, _ = spotUtils_ros.getPosition(self.pose_state)
            return math.hypot(target_map_x - robot_x, target_map_y - robot_y)

        if self.map_frame is None or self.odom_frame is None:
            return None

        try:
            target_ctrl_x, target_ctrl_y = self._transform_point_2d(
                target_map_x,
                target_map_y,
                self.map_frame,
                self.odom_frame,
            )
        except TransformException:
            return None

        robot_ctrl_x, robot_ctrl_y, _, _ = spotUtils_ros.getPosition(self.pose_state_odom)
        return math.hypot(target_ctrl_x - robot_ctrl_x, target_ctrl_y - robot_ctrl_y)

    def _safe_move_to_target(self, target_x, target_y, grid_helper):
        """Esegue avvicinamento incrementale con evitamento ostacoli e sidestep.

        Corrisponde alla parte "ruota + muovi" di `easy_walk.py::attempt_enter_cell_from_position`,
        ma con planner locale reattivo piu robusto.
        """
        if not bool(self.get_parameter('avoid_enabled').value):
            return self._move_to_world_target(target_x, target_y)

        occupied_threshold = int(self.get_parameter('avoid_occupied_threshold').value)
        treat_unknown_as_obstacle = bool(self.get_parameter('avoid_unknown_as_obstacle').value)
        step_size = max(0.2, float(self.get_parameter('avoid_step_size').value))
        lateral_offset = max(0.2, float(self.get_parameter('avoid_lateral_offset').value))
        max_retries = max(0, int(self.get_parameter('avoid_max_retries').value))
        pos_tol = float(self.get_parameter('pos_tolerance').value)
        near_goal_radius = float(self.get_parameter('near_goal_radius').value)
        effective_goal_tol = max(pos_tol, near_goal_radius)
        max_steps = max(10, int(self.get_parameter('avoid_max_steps').value))
        stall_window = max(3, int(self.get_parameter('avoid_stall_window').value))
        progress_eps = max(1e-4, float(self.get_parameter('avoid_progress_epsilon').value))
        goal_hysteresis = max(0.0, float(self.get_parameter('avoid_goal_hysteresis').value))

        retries = 0
        step_idx = 0
        stalled_steps = 0
        if bool(self.get_parameter('debug_attempts').value):
            self.get_logger().info(
                f"[AVOID_DBG] start_safe_move target=({target_x:.3f},{target_y:.3f}) "
                f"step_size={step_size:.2f} lateral_offset={lateral_offset:.2f} max_retries={max_retries} "
                f"goal_tol={effective_goal_tol:.3f} (pos_tol={pos_tol:.3f}, near_goal={near_goal_radius:.3f}) "
                f"max_steps={max_steps} stall_window={stall_window}"
            )

        while True:
            step_idx += 1
            if step_idx > max_steps:
                self.get_logger().debug(
                    f"[AVOID_DBG] max_steps_exceeded step={step_idx} max_steps={max_steps}"
                )
                return False, 'AVOIDANCE_MAX_STEPS', {
                    'step_idx': step_idx,
                    'max_steps': max_steps,
                    'target_x': round(target_x, 3),
                    'target_y': round(target_y, 3),
                }

            robot_x, robot_y, _, _ = spotUtils_ros.getPosition(self.pose_state)
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.hypot(dx, dy)
            control_distance = self._distance_to_target_in_control_frame(target_x, target_y)
            if distance <= effective_goal_tol or (
                    control_distance is not None and control_distance <= effective_goal_tol
            ):
                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().info(
                        f"[AVOID_DBG] target_reached step={step_idx} map_dist={distance:.3f} "
                        f"ctrl_dist={control_distance} tol={effective_goal_tol:.3f}"
                    )
                return True, 'OK', {}

            step = min(step_size, distance)
            heading = math.atan2(dy, dx)
            step_target_x = robot_x + step * math.cos(heading)
            step_target_y = robot_y + step * math.sin(heading)

            if bool(self.get_parameter('debug_attempts').value):
                self.get_logger().info(
                    f"[AVOID_DBG] step={step_idx} dist={distance:.3f} retries={retries}/{max_retries} "
                    f"robot=({robot_x:.3f},{robot_y:.3f}) step_target=({step_target_x:.3f},{step_target_y:.3f})"
                )

            path_clear = self._is_corridor_clear(
                grid_helper,
                robot_x,
                robot_y,
                step_target_x,
                step_target_y,
                occupied_threshold,
                treat_unknown_as_obstacle,
            )

            if path_clear:
                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().info(f"[AVOID_DBG] step={step_idx} path_clear=True")
                pre_dist = distance
                ok_move, reason, details = self._move_to_world_target(step_target_x, step_target_y)
                if not ok_move:
                    return False, reason, details

                x_after, y_after, _, _ = spotUtils_ros.getPosition(self.pose_state)
                post_dist = math.hypot(target_x - x_after, target_y - y_after)
                post_control_dist = self._distance_to_target_in_control_frame(target_x, target_y)
                improvement = pre_dist - post_dist
                if improvement <= progress_eps:
                    stalled_steps += 1
                else:
                    stalled_steps = 0

                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().info(
                        f"[AVOID_DBG] post_move step={step_idx} pre_dist={pre_dist:.3f} post_dist={post_dist:.3f} "
                        f"improvement={improvement:.3f} stalled_steps={stalled_steps}/{stall_window} "
                        f"post_ctrl_dist={post_control_dist}"
                    )

                if stalled_steps >= stall_window:
                    if post_control_dist is not None and post_control_dist <= (effective_goal_tol + goal_hysteresis):
                        self.get_logger().debug(
                            f"[AVOID_DBG] near_goal_hysteresis_accept post_map_dist={post_dist:.3f} "
                            f"post_ctrl_dist={post_control_dist:.3f} tol={effective_goal_tol:.3f} hyst={goal_hysteresis:.3f}"
                        )
                        return True, 'OK', {}
                    return False, 'AVOIDANCE_STALLED', {
                        'step_idx': step_idx,
                        'stalled_steps': stalled_steps,
                        'stall_window': stall_window,
                        'post_dist': round(post_dist, 3),
                        'post_control_dist': None if post_control_dist is None else round(post_control_dist, 3),
                    }
                continue

            if bool(self.get_parameter('debug_attempts').value):
                self.get_logger().debug(f"[AVOID_DBG] step={step_idx} path_clear=False obstacle_ahead")

            if retries >= max_retries:
                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().debug(
                        f"[AVOID_DBG] retries_exhausted step={step_idx} retries={retries}/{max_retries}"
                    )
                return False, 'AVOIDANCE_BLOCKED', {
                    'retries': retries,
                    'max_retries': max_retries,
                    'target_x': round(target_x, 3),
                    'target_y': round(target_y, 3),
                }

            # Try a short sidestep to the clearer side and then resume forward stepping.
            retries += 1
            perp_x = -math.sin(heading)
            perp_y = math.cos(heading)

            candidates = [
                ('LEFT', robot_x + perp_x * lateral_offset, robot_y + perp_y * lateral_offset),
                ('RIGHT', robot_x - perp_x * lateral_offset, robot_y - perp_y * lateral_offset),
            ]

            moved = False
            for side, lat_x, lat_y in candidates:
                clear_lateral = self._is_corridor_clear(
                    grid_helper,
                    robot_x,
                    robot_y,
                    lat_x,
                    lat_y,
                    occupied_threshold,
                    treat_unknown_as_obstacle,
                )
                clear_forward_from_lateral = self._is_corridor_clear(
                    grid_helper,
                    lat_x,
                    lat_y,
                    step_target_x,
                    step_target_y,
                    occupied_threshold,
                    treat_unknown_as_obstacle,
                )
                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().info(
                        f"[AVOID_DBG] step={step_idx} side={side} clear_lateral={clear_lateral} "
                        f"clear_forward_from_lateral={clear_forward_from_lateral} "
                        f"lat_target=({lat_x:.3f},{lat_y:.3f})"
                    )
                if not (clear_lateral and clear_forward_from_lateral):
                    continue

                self.get_logger().warn(
                    f"[AVOID] obstacle ahead, sidestep {side} by {lateral_offset:.2f}m (retry {retries}/{max_retries})"
                )
                ok_move, reason, details = self._move_to_world_target(lat_x, lat_y)
                if not ok_move:
                    return False, reason, details
                moved = True
                break

            if not moved:
                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().debug(
                        f"[AVOID_DBG] no_lateral_candidate step={step_idx} retries={retries}/{max_retries}"
                    )
                return False, 'AVOIDANCE_BLOCKED', {
                    'retries': retries,
                    'max_retries': max_retries,
                    'target_x': round(target_x, 3),
                    'target_y': round(target_y, 3),
                    'note': 'no_lateral_candidate',
                }

    def _publish_cmd(self, cmd: Twist):
        """Adapter publisher per `movements_ros.MotionController`."""
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        """Converte yaw (2D) in quaternione planare."""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return 0.0, 0.0, qz, qw

    @staticmethod
    def _quat_to_yaw(qx, qy, qz, qw):
        """Estrae lo yaw da un quaternione."""
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

    @staticmethod
    def _normalize_angle(a):
        """Normalizza angolo in [-pi, pi]."""
        return math.atan2(math.sin(a), math.cos(a))

    def _now_seconds(self):
        """Timestamp monotono in secondi dal clock ROS."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _update_pose_in_map_frame(self):
        """Mantiene `pose_state` nel frame mappa usando TF (fallback a odom se assente).

        Non ha equivalente diretto in `easy_walk.py` (lato SDK legge direttamente frame VISION).
        """
        if self.map_frame is None or self.odom_frame is None:
            return

        if self.map_frame == self.odom_frame:
            self.pose_state.x = self.pose_state_odom.x
            self.pose_state.y = self.pose_state_odom.y
            self.pose_state.z = self.pose_state_odom.z
            self.pose_state.qx = self.pose_state_odom.qx
            self.pose_state.qy = self.pose_state_odom.qy
            self.pose_state.qz = self.pose_state_odom.qz
            self.pose_state.qw = self.pose_state_odom.qw
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                self.last_odom_stamp if self.last_odom_stamp is not None else Time(),
            )
        except TransformException as exc:
            if not self._tf_warned:
                self.get_logger().warn(
                    f"[TF] map<-odom unavailable ({self.map_frame}<-{self.odom_frame}): {exc}. "
                    "Using odom pose until TF is ready."
                )
                self._tf_warned = True
            self.pose_state.x = self.pose_state_odom.x
            self.pose_state.y = self.pose_state_odom.y
            self.pose_state.z = self.pose_state_odom.z
            self.pose_state.qx = self.pose_state_odom.qx
            self.pose_state.qy = self.pose_state_odom.qy
            self.pose_state.qz = self.pose_state_odom.qz
            self.pose_state.qw = self.pose_state_odom.qw
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        rq = tf.transform.rotation
        tf_yaw = self._quat_to_yaw(rq.x, rq.y, rq.z, rq.w)

        ox = self.pose_state_odom.x
        oy = self.pose_state_odom.y
        oyaw = self.pose_state_odom.yaw()

        c = math.cos(tf_yaw)
        s = math.sin(tf_yaw)

        mx = tx + c * ox - s * oy
        my = ty + s * ox + c * oy
        myaw = self._normalize_angle(tf_yaw + oyaw)
        qx, qy, qz, qw = self._yaw_to_quaternion(myaw)

        self.pose_state.x = mx
        self.pose_state.y = my
        self.pose_state.z = self.pose_state_odom.z
        self.pose_state.qx = qx
        self.pose_state.qy = qy
        self.pose_state.qz = qz
        self.pose_state.qw = qw
        self._tf_warned = False

    def _on_odom(self, msg: Odometry):
        """Callback odometria: aggiorna stato posa in odom/map."""
        spotUtils_ros.update_pose_from_odom(self.pose_state_odom, msg)
        self.last_odom_stamp = Time.from_msg(msg.header.stamp)
        self.odom_frame = msg.header.frame_id.lstrip('/') or msg.header.frame_id
        self._update_pose_in_map_frame()

    def _on_map(self, msg: OccupancyGrid):
        """Callback mappa: salva OccupancyGrid corrente e frame planning."""
        self.current_map = msg
        self.map_frame = msg.header.frame_id.lstrip('/') or msg.header.frame_id
        self._update_pose_in_map_frame()
        if not self._map_debug_printed:
            self.get_logger().info(
                f"[MAP_DEBUG] First map received: {msg.info.width}x{msg.info.height}, "
                f"res={msg.info.resolution:.4f}, "
                f"origin=({msg.info.origin.position.x:.3f},{msg.info.origin.position.y:.3f}), "
                f"frame_id={msg.header.frame_id}, data_len={len(msg.data)}"
            )
            if self.odom_frame is not None and self.map_frame is not None:
                self.get_logger().info(
                    f"[TF_DEBUG] Planning frame={self.map_frame}, odom frame={self.odom_frame}"
                )
            self._map_debug_printed = True

    def _on_scan(self, msg: LaserScan):
        """Aggiorna una nuvola locale ostacoli in frame mappa dai raggi laser recenti.

        Questo layer locale rende l'evitamento più reattivo della sola OccupancyGrid globale.
        """
        if not bool(self.get_parameter('local_obs_enabled').value):
            return
        # Hard gate: scans are accepted only during the explicit waypoint burst window.
        if not self._lidar_burst_active:
            return
        if not self._accept_lidar_scans:
            return
        if self.map_frame is None:
            return

        from_frame = msg.header.frame_id.lstrip('/') or msg.header.frame_id
        if not from_frame:
            return

        stamp = Time.from_msg(msg.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, from_frame, stamp)
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, from_frame, Time())
            except TransformException:
                return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        rq = tf.transform.rotation
        tf_yaw = self._quat_to_yaw(rq.x, rq.y, rq.z, rq.w)
        c = math.cos(tf_yaw)
        s = math.sin(tf_yaw)

        stride = max(1, int(self.get_parameter('local_obs_stride').value))
        min_range = max(0.0, float(self.get_parameter('local_obs_min_range').value))
        max_range = max(min_range + 0.01, float(self.get_parameter('local_obs_max_range').value))

        points = []
        angle = msg.angle_min
        for idx, rng in enumerate(msg.ranges):
            if idx % stride != 0:
                angle += msg.angle_increment
                continue
            if not math.isfinite(rng) or rng < min_range or rng > max_range:
                angle += msg.angle_increment
                continue

            lx = rng * math.cos(angle)
            ly = rng * math.sin(angle)
            mx = tx + c * lx - s * ly
            my = ty + s * lx + c * ly
            points.append((mx, my))
            angle += msg.angle_increment

        self._local_obs_points_map = points
        self._local_obs_stamp_sec = self._now_seconds()

    def _on_camera_local_grid(self, msg: OccupancyGrid):
        """Aggiorna local grid di prossimita proveniente dalla pipeline camere (se disponibile)."""
        if not bool(self.get_parameter('camera_local_grid_enabled').value):
            return
        self._camera_local_grid_helper = spotGrid_ros.OccupancyGridHelper.from_msg(msg)
        self._camera_local_grid_stamp_sec = self._now_seconds()

    def _get_pose(self):
        """Restituisce posa corrente nel frame mappa."""
        return self.pose_state.x, self.pose_state.y, self.pose_state.z, self.pose_state.yaw()

    def _get_control_pose(self):
        """Restituisce posa nel frame usato dal controller di moto (map/odom)."""
        control_frame = str(self.get_parameter('control_frame').value).lower().strip()
        if control_frame == 'odom':
            return (
                self.pose_state_odom.x,
                self.pose_state_odom.y,
                self.pose_state_odom.z,
                self.pose_state_odom.yaw(),
            )
        return self._get_pose()

    def _spin_for_control(self, timeout_sec=0.0):
        """Pump ROS usato dal controller durante i loop di controllo."""
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def wait_for_data(self, timeout=10.0):
        """Attende l'arrivo della prima mappa prima di partire."""
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            if self.current_map is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def attempt_enter_cell(self, env, target_row, target_col):
        """Tenta ingresso nella cella target partendo dalla posa corrente.

        Corrisponde a `easy_walk.py::attempt_enter_cell_from_position`.
        """
        if self.current_map is None:
            self.get_logger().error('No OccupancyGrid received yet.')
            return False, 'NO_MAP', {}

        grid_helper, _, _ = spotGrid_ros.create_obstacle_grid_from_occupancy(self.current_map)
        robot_x, robot_y, _, _ = spotUtils_ros.getPosition(self.pose_state)
        if bool(self.get_parameter('debug_attempts').value):
            self.get_logger().debug(
                f"[SDK_PARITY] attempt_enter_cell start cell=({target_row},{target_col}) robot=({robot_x:.3f},{robot_y:.3f})"
            )
        iteration = self.visualization_counter
        self.visualization_counter += 1
        self._log_alignment_debug(env, 'ALIGN_PRE', robot_x, robot_y, target_row, target_col)

        target_x, target_y, valid_samples, rejected_samples, los_stats = find_best_point_in_cell(
            robot_x,
            robot_y,
            env,
            target_row,
            target_col,
            grid_helper,
            occupied_threshold=int(self.get_parameter('occupied_threshold').value),
            max_occupied_ratio=float(self.get_parameter('max_occupied_ratio').value),
            min_consecutive_hits=int(self.get_parameter('min_consecutive_hits').value),
        )
        self._log_sample_decision(
            f"cell=({target_row},{target_col}) samples_total={los_stats['total_samples']} "
            f"valid={los_stats['valid_samples']} rejected={los_stats['rejected_samples']} "
            f"reasons={los_stats['reject_reasons']} avg_rej_occ_ratio={los_stats['avg_rejected_occupied_ratio']:.3f}"
        )
        candidates_payload = {
            'rejected_los': list(rejected_samples),
            'valid_los': list(valid_samples),
            'rejected_corridor': [],
            'valid_corridor': [],
        }
        dump_limit = max(0, int(self.get_parameter('sample_rejected_dump_limit').value))
        rejected_dump = list(los_stats.get('rejected_dump', []))
        if dump_limit > 0 and rejected_dump:
            for i, item in enumerate(rejected_dump[:dump_limit], start=1):
                self._log_sample_decision(
                    f"cell=({target_row},{target_col}) rejected[{i}/{len(rejected_dump)}] "
                    f"sample=({item['x']:.3f},{item['y']:.3f}) reason={item['reason']} "
                    f"occ_ratio={item['occupied_ratio']:.3f} hits={item['occupied_hits']}/{item['valid_checks']}"
                )

        if target_x is None or target_y is None:
            self._log_sample_decision(
                f"cell=({target_row},{target_col}) decision=DISCARD reason=LOS_BLOCKED "
                f"valid={len(valid_samples)} rejected={len(rejected_samples)}"
            )
            visualize_grid_with_candidates_ros(
                self,
                grid_helper,
                env,
                robot_x,
                robot_y,
                candidates_payload,
                None,
                iteration,
            )
            return False, 'LOS_BLOCKED', {
                'valid_samples': len(valid_samples),
                'rejected_samples': len(rejected_samples),
                'robot_x': round(robot_x, 3),
                'robot_y': round(robot_y, 3),
                'target_row': target_row,
                'target_col': target_col,
            }

        cell_center = env.get_world_position_from_cell(target_row, target_col)
        corridor_valid = []
        corridor_rejected = []
        if cell_center is None:
            ordered_valid_samples = list(valid_samples)
        else:
            cx, cy = cell_center
            ordered_valid_samples = sorted(
                valid_samples,
                key=lambda p: math.hypot(p[0] - cx, p[1] - cy),
            )

        avoid_occ_threshold = int(self.get_parameter('avoid_occupied_threshold').value)
        avoid_unknown_as_obstacle = bool(self.get_parameter('avoid_unknown_as_obstacle').value)
        for sx, sy in ordered_valid_samples:
            if self._is_corridor_clear(
                    grid_helper,
                    robot_x,
                    robot_y,
                    sx,
                    sy,
                    avoid_occ_threshold,
                    avoid_unknown_as_obstacle,
            ):
                corridor_valid.append((sx, sy))
            else:
                corridor_rejected.append((sx, sy))

        candidates_payload['valid_corridor'] = list(corridor_valid)
        candidates_payload['rejected_corridor'] = list(corridor_rejected)

        self._log_sample_decision(
            f"cell=({target_row},{target_col}) corridor_filter valid={len(corridor_valid)} "
            f"rejected={len(corridor_rejected)}"
        )
        corridor_dump_limit = max(0, int(self.get_parameter('sample_corridor_dump_limit').value))
        if corridor_dump_limit > 0 and corridor_rejected:
            for i, (sx, sy) in enumerate(corridor_rejected[:corridor_dump_limit], start=1):
                self._log_sample_decision(
                    f"cell=({target_row},{target_col}) corridor_rejected[{i}/{len(corridor_rejected)}] "
                    f"sample=({sx:.3f},{sy:.3f})"
                )

        if not corridor_valid:
            self._log_sample_decision(
                f"cell=({target_row},{target_col}) decision=DISCARD reason=CORRIDOR_BLOCKED "
                f"valid_los={len(valid_samples)} rejected_corridor={len(corridor_rejected)}"
            )
            visualize_grid_with_candidates_ros(
                self,
                grid_helper,
                env,
                robot_x,
                robot_y,
                candidates_payload,
                None,
                iteration,
            )
            return False, 'CORRIDOR_BLOCKED', {
                'valid_samples': len(valid_samples),
                'corridor_rejected': len(corridor_rejected),
                'target_row': target_row,
                'target_col': target_col,
            }

        target_x, target_y = corridor_valid[0]

        if self._has_fresh_local_observation() and self._is_local_obstacle_near(target_x, target_y):
            if cell_center is not None:
                cx, cy = cell_center
                locally_safe = [
                    p for p in corridor_valid
                    if not self._is_local_obstacle_near(p[0], p[1])
                ]
                blocked_by_local_obs = max(0, len(corridor_valid) - len(locally_safe))
                if locally_safe:
                    old_target_x = target_x
                    old_target_y = target_y
                    target_x, target_y = min(
                        locally_safe,
                        key=lambda p: math.hypot(p[0] - cx, p[1] - cy),
                    )
                    self._log_sample_decision(
                        f"cell=({target_row},{target_col}) local_obs_refine blocked_valid={blocked_by_local_obs}/{len(corridor_valid)} "
                        f"target_before=({old_target_x:.3f},{old_target_y:.3f}) target_after=({target_x:.3f},{target_y:.3f})"
                    )
                else:
                    self._log_sample_decision(
                        f"cell=({target_row},{target_col}) decision=DISCARD reason=LOCAL_OBS_BLOCKED "
                        f"blocked_valid={blocked_by_local_obs}/{len(corridor_valid)}"
                    )
                    visualize_grid_with_candidates_ros(
                        self,
                        grid_helper,
                        env,
                        robot_x,
                        robot_y,
                        candidates_payload,
                        None,
                        iteration,
                    )
                    return False, 'LOCAL_OBS_BLOCKED', {
                        'target_row': target_row,
                        'target_col': target_col,
                        'valid_samples': len(valid_samples),
                        'reason': 'local_observation_blocks_all_candidates',
                    }

        chosen_los = los_stats.get('chosen_los_sample')
        if chosen_los is not None:
            self._log_sample_decision(
                f"cell=({target_row},{target_col}) chosen_los=({chosen_los['x']:.3f},{chosen_los['y']:.3f}) "
                f"dist_center={chosen_los['distance_to_center']:.3f}"
            )
        self._log_sample_decision(
            f"cell=({target_row},{target_col}) chosen_target=({target_x:.3f},{target_y:.3f})"
        )

        if bool(self.get_parameter('debug_attempts').value):
            self.get_logger().info(
                f"[ATTEMPT] cell=({target_row},{target_col}) target=({target_x:.3f},{target_y:.3f}) "
                f"samples_valid={len(valid_samples)} samples_rejected={len(rejected_samples)}"
            )

        visualize_grid_with_candidates_ros(
            self,
            grid_helper,
            env,
            robot_x,
            robot_y,
            candidates_payload,
            (target_x, target_y),
            iteration,
        )

        distance = math.hypot(target_x - robot_x, target_y - robot_y)
        if bool(self.get_parameter('debug_attempts').value):
            self.get_logger().info(f"[CTRL] frame=map target=({target_x:.3f},{target_y:.3f}) dist={distance:.2f}m")

        ok_move, reason, details = self._safe_move_to_target(target_x, target_y, grid_helper)
        if not ok_move:
            if bool(self.get_parameter('debug_attempts').value):
                self.get_logger().debug(
                    f"[SDK_PARITY] attempt_enter_cell move_failed cell=({target_row},{target_col}) reason={reason} details={details}"
                )
            return False, reason, details

        x_final, y_final, _, _ = spotUtils_ros.getPosition(self.pose_state)
        self._log_alignment_debug(env, 'ALIGN_POST', x_final, y_final, target_row, target_col)
        env.add_robot_position(x_final, y_final, movement_type='explore')
        inside = env.is_point_in_cell(x_final, y_final, target_row, target_col)
        if not inside:
            if bool(self.get_parameter('debug_attempts').value):
                self.get_logger().debug(
                    f"[SDK_PARITY] attempt_enter_cell cell_mismatch target=({target_row},{target_col}) final=({x_final:.3f},{y_final:.3f})"
                )
            return False, 'CELL_MISMATCH', {
                'final_x': round(x_final, 3),
                'final_y': round(y_final, 3),
                'target_row': target_row,
                'target_col': target_col,
            }
        if bool(self.get_parameter('debug_attempts').value):
            self.get_logger().debug(
                f"[SDK_PARITY] attempt_enter_cell success cell=({target_row},{target_col})"
            )
        return True, 'OK', {}

    def run(self):
        """Loop principale di esplorazione su frontier in stile serpentina.

        Corrisponde a `easy_walk.py::easy_walk` (sezione mission loop),
        senza dipendenze dirette dal GraphNav runtime.
        """
        if not self.wait_for_data():
            self.get_logger().error('Timeout waiting for map data.')
            return

        env = EnvironmentMap(
            rows=self.get_parameter('grid_rows').value,
            cols=self.get_parameter('grid_cols').value,
            cell_size=self.get_parameter('cell_size').value,
        )

        x_boot, y_boot, z_boot, _ = spotUtils_ros.getPosition(self.pose_state)
        yaw_boot = self.pose_state.yaw()
        env.set_origin(x_boot, y_boot, yaw_boot, start_row=0, start_col=0)

        recording = navGraphUtils_ros.RecordingInterface()
        self.get_logger().warn(
            '[NAVGRAPH] Using ROS stub RecordingInterface: GraphNav navigation is not active in this mode.'
        )
        recording.clear_map()
        recording.create_default_waypoint(cell_row=0, cell_col=0, x=x_boot, y=y_boot, z=z_boot, yaw=yaw_boot)
        env.add_waypoint(x_boot, y_boot)
        env.add_robot_position(x_boot, y_boot, movement_type='explore')
        self._capture_lidar_snapshot_at_waypoint(label='bootstrap')

        path = env.generate_serpentine_path(start_cell=env.start_cell)
        frontier = []

        robot_row, robot_col = env.get_cell_from_world(x_boot, y_boot)
        frontier = self._merge_frontier(
            frontier,
            env.get_adjacent_frontier_cells(robot_row, robot_col, path),
            reason='bootstrap',
        )

        while rclpy.ok():
            self._log_frontier(frontier, 'loop start (sorted)')
            if len(frontier) == 0:
                break

            x, y, _, _ = spotUtils_ros.getPosition(self.pose_state)
            robot_row, robot_col = env.get_cell_from_world(x, y)
            borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)

            borders_in_frontier = []
            for border in borders:
                if any((f[0] == border[0] and f[1] == border[1]) for f in frontier):
                    borders_in_frontier.append(border)

            if len(borders_in_frontier) != 0:
                selected_border = min(borders_in_frontier, key=lambda b: b[2])
                self._log_frontier_decision(
                    f"choose=ADJACENT cell=({selected_border[0]},{selected_border[1]}) rank={selected_border[2]}"
                )
                if bool(self.get_parameter('frontier_verbose').value):
                    self.get_logger().info(
                        f"[FRONTIER] selected adjacent cell=({selected_border[0]},{selected_border[1]}) rank={selected_border[2]}"
                    )
                ok, reason, details = self.attempt_enter_cell(env, selected_border[0], selected_border[1])
                self._log_frontier_decision(
                    f"attempt adjacent cell=({selected_border[0]},{selected_border[1]}) "
                    f"ok={ok} reason={reason} details={details}"
                )
                frontier = self._remove_frontier_cell(
                    frontier,
                    selected_border[0],
                    selected_border[1],
                    reason='selected_adjacent_attempted',
                )

                if ok:
                    x_new, y_new = self._finalize_successful_cell_visit(
                        env,
                        recording,
                        selected_border[0],
                        selected_border[1],
                    )

                    robot_row, robot_col = env.get_cell_from_world(x_new, y_new)
                    frontier = self._merge_frontier(
                        frontier,
                        env.get_adjacent_frontier_cells(robot_row, robot_col, path),
                        reason=f'from_visited_{selected_border[0]}_{selected_border[1]}',
                    )
                else:
                    frontier = self._handle_failed_attempt(
                        env,
                        frontier,
                        selected_border[0],
                        selected_border[1],
                        selected_border[2],
                        reason,
                        details,
                        attempt_kind='adjacent',
                    )
            else:
                frontier_sorted = self._sort_frontier(frontier)
                global_best = frontier_sorted[0] if frontier_sorted else None
                self._log_frontier_decision(
                    f"no_adjacent_frontier robot_cell=({robot_row},{robot_col}) "
                    f"adjacent={self._format_cells(borders)} "
                    f"adjacent_in_frontier={self._format_cells(borders_in_frontier)} "
                    f"global_best={global_best}"
                )
                lowest_rank_cell = env.get_lowest_rank_from_frontier_list(frontier, path)
                if lowest_rank_cell is None:
                    break
                target_row, target_col, rank = lowest_rank_cell
                self._log_frontier_decision(
                    f"choose=FALLBACK reason=no_adjacent_frontier target=({target_row},{target_col}) rank={rank}"
                )
                if bool(self.get_parameter('frontier_verbose').value):
                    self.get_logger().info(
                        f"[FRONTIER] selected fallback cell=({target_row},{target_col}) rank={rank}"
                    )
                ok, reason, details = self.attempt_enter_cell(env, target_row, target_col)
                self._log_frontier_decision(
                    f"attempt fallback cell=({target_row},{target_col}) ok={ok} reason={reason} details={details}"
                )
                frontier = self._remove_frontier_cell(
                    frontier,
                    target_row,
                    target_col,
                    reason='fallback_attempted',
                )
                if ok:
                    x_new, y_new = self._finalize_successful_cell_visit(
                        env,
                        recording,
                        target_row,
                        target_col,
                    )
                    frontier = self._merge_frontier(
                        frontier,
                        env.get_adjacent_frontier_cells(*env.get_cell_from_world(x_new, y_new), path),
                        reason=f'from_visited_{target_row}_{target_col}',
                    )
                else:
                    frontier = self._handle_failed_attempt(
                        env,
                        frontier,
                        target_row,
                        target_col,
                        rank,
                        reason,
                        details,
                        attempt_kind='fallback',
                    )

            rclpy.spin_once(self, timeout_sec=0.1)

        self._return_to_start_via_robot_path(env)
        self.get_logger().info('Exploration complete.')


def main():
    """Entry-point ROS del nodo Easy Walk."""
    rclpy.init()
    node = EasyWalkROS()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

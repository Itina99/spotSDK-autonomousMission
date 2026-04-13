#!/usr/bin/env python3
import os
import time
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
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
    """Conservative LOS check to reduce false blocked cells from sparse map noise."""
    distance = math.hypot(x2 - x1, y2 - y1)
    if distance < 1e-6:
        return True

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
                return False
        else:
            consecutive_hits = 0

    if valid_checks == 0:
        return True

    return (occupied_hits / float(valid_checks)) <= max_occupied_ratio


def sample_cell_points(env, cell_row, cell_col, num_samples=200):
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
    sampled_points = sample_cell_points(env, cell_row, cell_col, num_samples=100)
    if not sampled_points:
        return None, None, [], []

    cell_center = env.get_world_position_from_cell(cell_row, cell_col)
    if cell_center is None:
        return None, None, [], []

    cell_center_x, cell_center_y = cell_center

    valid_samples = []
    rejected_samples = []

    for sample_x, sample_y in sampled_points:
        if check_line_of_sight(
            robot_x,
            robot_y,
            sample_x,
            sample_y,
            grid_helper,
            occupied_threshold=occupied_threshold,
            max_occupied_ratio=max_occupied_ratio,
            min_consecutive_hits=min_consecutive_hits,
        ):
            valid_samples.append((sample_x, sample_y))
        else:
            rejected_samples.append((sample_x, sample_y))

    if not valid_samples:
        return None, None, valid_samples, rejected_samples

    best_point = None
    min_distance = float('inf')

    for sample_x, sample_y in valid_samples:
        dist = math.hypot(sample_x - cell_center_x, sample_y - cell_center_y)
        if dist < min_distance:
            min_distance = dist
            best_point = (sample_x, sample_y)

    return best_point[0], best_point[1], valid_samples, rejected_samples


def visualize_grid_with_candidates_ros(helper, env, robot_x, robot_y, candidates, chosen_point, iteration, save_path=None):
    # Matplotlib output removed; this hook is intentionally quiet.
    return


class EasyWalkROS(Node):
    def __init__(self):
        super().__init__('easy_walk_ros')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('grid_rows', 10)
        self.declare_parameter('grid_cols', 10)
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
        self.declare_parameter('moving_angular_scale', 0.4)
        self.declare_parameter('moving_angular_cap', 0.25)
        self.declare_parameter('frontier_verbose', False)
        self.declare_parameter('debug_attempts', True)
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
        self.declare_parameter('avoid_corridor_half_width', 0.22)
        self.declare_parameter('avoid_retreat_steps', 2)
        self.declare_parameter('avoid_max_steps', 250)
        self.declare_parameter('avoid_stall_window', 8)
        self.declare_parameter('avoid_progress_epsilon', 0.01)
        self.declare_parameter('avoid_goal_hysteresis', 0.03)

        map_topic = self.get_parameter('map_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.pose_state = spotUtils_ros.PoseState()
        self.pose_state_odom = spotUtils_ros.PoseState()
        self.last_odom_stamp = None
        self.map_frame = None
        self.odom_frame = None
        self._tf_warned = False
        self.current_map = None
        self._map_debug_printed = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 1)

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
        return f"{row},{col}"

    def _is_transient_failure(self, reason):
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

    def _handle_failed_attempt(self, env, frontier, row, col, rank, reason, details):
        key = self._cell_key(row, col)
        debug_attempts = bool(self.get_parameter('debug_attempts').value)

        retreat_reasons = {'MOVE_NO_PROGRESS', 'AVOIDANCE_BLOCKED', 'AVOIDANCE_STALLED'}
        details_payload = dict(details) if isinstance(details, dict) else {'raw_details': str(details)}
        if reason in retreat_reasons:
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
        return sorted(frontier, key=lambda c: (c[2], c[0], c[1]))

    def _log_frontier(self, frontier, label):
        if not bool(self.get_parameter('frontier_verbose').value):
            return
        ordered = self._sort_frontier(frontier)
        if not ordered:
            self.get_logger().info(f"[FRONTIER] {label}: empty")
            return
        cells = ', '.join([f"({r},{c},rank={rank})" for r, c, rank in ordered])
        self.get_logger().info(f"[FRONTIER] {label}: {len(ordered)} cells -> [{cells}]")

    def _merge_frontier(self, frontier, candidates, reason):
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
        before = len(frontier)
        frontier = [f for f in frontier if not (f[0] == row and f[1] == col)]
        removed = before - len(frontier)
        self.get_logger().info(
            f"[FRONTIER] removed ({reason}): cell=({row},{col}) removed_count={removed}"
        )
        self._log_frontier(frontier, f"state after remove ({reason})")
        return frontier

    def _return_to_start_via_robot_path(self, env):
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
        distance = math.hypot(x2 - x1, y2 - y1)
        if distance < 1e-6:
            return True

        sample_step = max(0.03, float(self.get_parameter('avoid_sample_step').value))
        samples = max(3, int(math.ceil(distance / sample_step)))

        for i in range(1, samples + 1):
            t = i / float(samples)
            sx = x1 + (x2 - x1) * t
            sy = y1 + (y2 - y1) * t
            if grid_helper.is_occupied(
                sx,
                sy,
                threshold=occupied_threshold,
                treat_unknown_as_obstacle=treat_unknown_as_obstacle,
            ):
                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().info(
                        f"[AVOID_DBG] segment_blocked sample={i}/{samples} at ({sx:.3f},{sy:.3f}) "
                        f"from=({x1:.3f},{y1:.3f}) to=({x2:.3f},{y2:.3f})"
                    )
                return False
        return True

    def _is_corridor_clear(self, grid_helper, x1, y1, x2, y2, occupied_threshold, treat_unknown_as_obstacle):
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
                        self.get_logger().warn(
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
                    self.get_logger().warn(f"[AVOID_DBG] transform_exception: {exc}")
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
                self.get_logger().warn(
                    f"[AVOID_DBG] move_failed reason={self.motion.last_failure_reason} "
                    f"details={self.motion.last_failure_details}"
                )
            return False, self.motion.last_failure_reason, dict(self.motion.last_failure_details)

        if bool(self.get_parameter('debug_attempts').value):
            self.get_logger().info("[AVOID_DBG] move_ok")
        return True, 'OK', {}

    def _distance_to_target_in_control_frame(self, target_map_x, target_map_y):
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
                self.get_logger().warn(
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
                        self.get_logger().warn(
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
                self.get_logger().warn(f"[AVOID_DBG] step={step_idx} path_clear=False obstacle_ahead")

            if retries >= max_retries:
                if bool(self.get_parameter('debug_attempts').value):
                    self.get_logger().warn(
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
                    self.get_logger().warn(
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
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return 0.0, 0.0, qz, qw

    @staticmethod
    def _quat_to_yaw(qx, qy, qz, qw):
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

    @staticmethod
    def _normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _update_pose_in_map_frame(self):
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
        spotUtils_ros.update_pose_from_odom(self.pose_state_odom, msg)
        self.last_odom_stamp = Time.from_msg(msg.header.stamp)
        self.odom_frame = msg.header.frame_id.lstrip('/') or msg.header.frame_id
        self._update_pose_in_map_frame()

    def _on_map(self, msg: OccupancyGrid):
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

    def _get_pose(self):
        return self.pose_state.x, self.pose_state.y, self.pose_state.z, self.pose_state.yaw()

    def _get_control_pose(self):
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
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def wait_for_data(self, timeout=10.0):
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            if self.current_map is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def attempt_enter_cell(self, env, target_row, target_col):
        if self.current_map is None:
            self.get_logger().error('No OccupancyGrid received yet.')
            return False, 'NO_MAP', {}

        grid_helper, _, _ = spotGrid_ros.create_obstacle_grid_from_occupancy(self.current_map)
        robot_x, robot_y, _, _ = spotUtils_ros.getPosition(self.pose_state)
        self._log_alignment_debug(env, 'ALIGN_PRE', robot_x, robot_y, target_row, target_col)

        target_x, target_y, valid_samples, rejected_samples = find_best_point_in_cell(
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

        if target_x is None or target_y is None:
            return False, 'LOS_BLOCKED', {
                'valid_samples': len(valid_samples),
                'rejected_samples': len(rejected_samples),
                'robot_x': round(robot_x, 3),
                'robot_y': round(robot_y, 3),
                'target_row': target_row,
                'target_col': target_col,
            }

        if bool(self.get_parameter('debug_attempts').value):
            self.get_logger().info(
                f"[ATTEMPT] cell=({target_row},{target_col}) target=({target_x:.3f},{target_y:.3f}) "
                f"samples_valid={len(valid_samples)} samples_rejected={len(rejected_samples)}"
            )

        distance = math.hypot(target_x - robot_x, target_y - robot_y)
        self.get_logger().info(f"[CTRL] frame=map target=({target_x:.3f},{target_y:.3f}) dist={distance:.2f}m")

        ok_move, reason, details = self._safe_move_to_target(target_x, target_y, grid_helper)
        if not ok_move:
            return False, reason, details

        x_final, y_final, _, _ = spotUtils_ros.getPosition(self.pose_state)
        self._log_alignment_debug(env, 'ALIGN_POST', x_final, y_final, target_row, target_col)
        env.add_robot_position(x_final, y_final, movement_type='explore')
        inside = env.is_point_in_cell(x_final, y_final, target_row, target_col)
        if not inside:
            return False, 'CELL_MISMATCH', {
                'final_x': round(x_final, 3),
                'final_y': round(y_final, 3),
                'target_row': target_row,
                'target_col': target_col,
            }
        return True, 'OK', {}

    def run(self):
        if not self.wait_for_data():
            self.get_logger().error('Timeout waiting for map data.')
            return

        env = EnvironmentMap(
            rows=self.get_parameter('grid_rows').value,
            cols=self.get_parameter('grid_cols').value,
            cell_size=self.get_parameter('cell_size').value,
        )

        x_boot, y_boot, z_boot, quat = spotUtils_ros.getPosition(self.pose_state)
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
                self.get_logger().info(
                    f"[FRONTIER] selected adjacent cell=({selected_border[0]},{selected_border[1]}) rank={selected_border[2]}"
                )
                ok, reason, details = self.attempt_enter_cell(env, selected_border[0], selected_border[1])
                frontier = self._remove_frontier_cell(
                    frontier,
                    selected_border[0],
                    selected_border[1],
                    reason='selected_adjacent_attempted',
                )

                if ok:
                    env.mark_cell_visited(selected_border[0], selected_border[1])
                    x_new, y_new, z_new, _ = spotUtils_ros.getPosition(self.pose_state)
                    recording.create_default_waypoint(cell_row=selected_border[0], cell_col=selected_border[1], x=x_new, y=y_new, z=z_new, yaw=self.pose_state.yaw())
                    env.add_waypoint(x_new, y_new)

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
                    )
            else:
                lowest_rank_cell = env.get_lowest_rank_from_frontier_list(frontier, path)
                if lowest_rank_cell is None:
                    break
                target_row, target_col, rank = lowest_rank_cell
                self.get_logger().info(
                    f"[FRONTIER] selected fallback cell=({target_row},{target_col}) rank={rank}"
                )
                ok, reason, details = self.attempt_enter_cell(env, target_row, target_col)
                frontier = self._remove_frontier_cell(
                    frontier,
                    target_row,
                    target_col,
                    reason='fallback_attempted',
                )
                if ok:
                    env.mark_cell_visited(target_row, target_col)
                    x_new, y_new, _, _ = spotUtils_ros.getPosition(self.pose_state)
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
                    )

            rclpy.spin_once(self, timeout_sec=0.1)

        self._return_to_start_via_robot_path(env)
        self.get_logger().info('Exploration complete.')


def main():
    rclpy.init()
    node = EasyWalkROS()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

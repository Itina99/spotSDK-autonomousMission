import math
import time
from geometry_msgs.msg import Twist


class MotionController:
    def __init__(self, get_pose_fn, publish_cmd_fn,
                 linear_speed=0.3, angular_speed=0.5,
                 yaw_tolerance=0.08, pos_tolerance=0.05,
                 spin_once_fn=None, now_fn=None,
                 control_period=0.05, timeout_scale=4.0,
                 no_progress_timeout=8.0, progress_epsilon=0.003,
                 linear_kp=0.8, angular_kp=1.0,
                 max_angular_accel=1.5,
                 move_yaw_blend_start=0.25,
                 move_yaw_blend_stop=0.9,
                 near_goal_radius=0.15,
                 near_goal_timeout_boost=2.0,
                 near_goal_yaw_relax=0.35,
                 moving_angular_scale=0.4,
                 moving_angular_cap=0.25):
        self._get_pose = get_pose_fn
        self._publish_cmd = publish_cmd_fn
        self._spin_once = spin_once_fn
        self._now = now_fn if now_fn is not None else time.monotonic
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.yaw_tolerance = yaw_tolerance
        self.pos_tolerance = pos_tolerance
        self.control_period = control_period
        self.timeout_scale = timeout_scale
        self.no_progress_timeout = no_progress_timeout
        self.progress_epsilon = progress_epsilon
        self.linear_kp = linear_kp
        self.angular_kp = angular_kp
        self.max_angular_accel = max_angular_accel
        self.move_yaw_blend_start = move_yaw_blend_start
        self.move_yaw_blend_stop = move_yaw_blend_stop
        self.near_goal_radius = near_goal_radius
        self.near_goal_timeout_boost = near_goal_timeout_boost
        self.near_goal_yaw_relax = near_goal_yaw_relax
        self.moving_angular_scale = moving_angular_scale
        self.moving_angular_cap = moving_angular_cap
        self.last_failure_reason = 'INIT'
        self.last_failure_details = {}
        self._last_cmd_w = 0.0

    def _tick(self):
        if self._spin_once is not None:
            self._spin_once(0.0)

    def _wait_step(self):
        if self._spin_once is not None:
            self._spin_once(self.control_period)
        else:
            time.sleep(self.control_period)

    def _stop(self):
        self._publish_cmd(Twist())

    def _set_failure(self, reason, **details):
        self.last_failure_reason = reason
        self.last_failure_details = details

    def _set_success(self, mode, **details):
        self.last_failure_reason = 'OK'
        payload = {'mode': mode}
        payload.update(details)
        self.last_failure_details = payload

    def rotate_by(self, dyaw, timeout=10.0):
        est_timeout = max(
            timeout,
            (abs(dyaw) / max(1e-3, self.angular_speed)) * self.timeout_scale + 1.0,
        )
        start = self._now()
        _, _, _, yaw = self._get_pose()
        target = yaw + dyaw

        while self._now() - start < est_timeout:
            self._tick()
            _, _, _, yaw = self._get_pose()
            err = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
            if abs(err) <= self.yaw_tolerance:
                self._stop()
                self._set_success('rotate', final_yaw=yaw, target_yaw=target, final_err=err)
                return True

            cmd = Twist()
            w = min(self.angular_speed, max(0.1, abs(err) * 0.8))
            cmd.angular.z = w if err > 0 else -w
            self._publish_cmd(cmd)
            self._wait_step()

        self._stop()
        _, _, _, yaw_end = self._get_pose()
        final_err = math.atan2(math.sin(target - yaw_end), math.cos(target - yaw_end))
        self._set_failure(
            'ROTATE_TIMEOUT',
            dyaw=dyaw,
            timeout=est_timeout,
            final_err=final_err,
        )
        return False

    def move_to(self, target_x, target_y, timeout=30.0):
        x0, y0, _, _ = self._get_pose()
        nominal_dist = math.hypot(target_x - x0, target_y - y0)
        est_timeout = max(
            timeout,
            (nominal_dist / max(1e-3, self.linear_speed)) * self.timeout_scale + 2.0,
        )

        min_dist_seen = nominal_dist
        last_progress_time = self._now()

        start = self._now()
        while self._now() - start < est_timeout:
            self._tick()
            x, y, _, yaw = self._get_pose()
            dx = target_x - x
            dy = target_y - y
            dist = math.hypot(dx, dy)
            if dist <= self.pos_tolerance or dist <= self.near_goal_radius:
                self._stop()
                self._set_success('move', target_x=target_x, target_y=target_y, final_dist=dist)
                return True

            if dist < (min_dist_seen - self.progress_epsilon):
                min_dist_seen = dist
                last_progress_time = self._now()

            desired_yaw = math.atan2(dy, dx)
            yaw_err = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))

            # During heading-only alignment, distance may stay almost constant; don't classify as stuck.
            if abs(yaw_err) > self.yaw_tolerance:
                last_progress_time = self._now()

            elapsed_since_progress = self._now() - last_progress_time
            elapsed_total = self._now() - start
            current_no_progress_timeout = self.no_progress_timeout
            if dist <= self.near_goal_radius:
                current_no_progress_timeout *= self.near_goal_timeout_boost

            if elapsed_total > 2.0 and elapsed_since_progress > current_no_progress_timeout:
                self._stop()
                self._set_failure(
                    'MOVE_NO_PROGRESS',
                    target_x=target_x,
                    target_y=target_y,
                    current_x=x,
                    current_y=y,
                    dist=dist,
                    min_dist_seen=min_dist_seen,
                    elapsed_total=elapsed_total,
                    elapsed_since_progress=elapsed_since_progress,
                )
                return False


            cmd = Twist()

            # Smooth angular control to reduce oscillation/jitter.
            w_des = max(-self.angular_speed, min(self.angular_speed, self.angular_kp * yaw_err))
            max_dw = self.max_angular_accel * self.control_period
            if w_des > self._last_cmd_w + max_dw:
                w_des = self._last_cmd_w + max_dw
            elif w_des < self._last_cmd_w - max_dw:
                w_des = self._last_cmd_w - max_dw
            cmd.angular.z = w_des
            self._last_cmd_w = w_des

            # Blend forward speed with heading error so robot does not rotate forever in place.
            heading_abs = abs(yaw_err)
            if heading_abs <= self.move_yaw_blend_start:
                heading_scale = 1.0
            elif heading_abs >= self.move_yaw_blend_stop:
                heading_scale = 0.0
            else:
                span = max(1e-6, self.move_yaw_blend_stop - self.move_yaw_blend_start)
                heading_scale = 1.0 - ((heading_abs - self.move_yaw_blend_start) / span)

            v_nominal = min(self.linear_speed, max(0.0, dist * self.linear_kp))
            v_des = v_nominal * heading_scale

            if dist <= self.near_goal_radius:
                v_des = min(v_des, 0.12)
                if heading_abs <= self.near_goal_yaw_relax:
                    v_des = max(v_des, 0.04)

            if v_des > 0.0:
                cmd.linear.x = max(0.03, v_des)
                # Gazebo locomotion can over-rotate while walking; damp yaw when translating.
                cmd.angular.z = max(
                    -self.moving_angular_cap,
                    min(self.moving_angular_cap, cmd.angular.z * self.moving_angular_scale),
                )

            self._publish_cmd(cmd)
            self._wait_step()

        self._stop()
        x_end, y_end, _, _ = self._get_pose()
        final_dist = math.hypot(target_x - x_end, target_y - y_end)
        self._set_failure(
            'MOVE_TIMEOUT',
            target_x=target_x,
            target_y=target_y,
            timeout=est_timeout,
            final_dist=final_dist,
        )
        return False

    def relative_move(self, dx, dy, dyaw):
        x, y, _, yaw = self._get_pose()

        if abs(dx) < 1e-6 and abs(dy) < 1e-6 and abs(dyaw) > 1e-6:
            return self.rotate_by(dyaw), 0.0

        target_x = x + dx * math.cos(yaw) - dy * math.sin(yaw)
        target_y = y + dx * math.sin(yaw) + dy * math.cos(yaw)
        ok = self.move_to(target_x, target_y)
        dist = math.hypot(target_x - x, target_y - y)
        return ok, dist


def relative_move(dx, dy, dyaw, controller):
    """Convenience wrapper mirroring the Spot SDK signature pattern."""
    return controller.relative_move(dx, dy, dyaw)

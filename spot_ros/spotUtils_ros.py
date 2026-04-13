import math
from dataclasses import dataclass


@dataclass
class PoseState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def yaw(self) -> float:
        return math.atan2(
            2.0 * (self.qw * self.qz + self.qx * self.qy),
            1.0 - 2.0 * (self.qy ** 2 + self.qz ** 2)
        )


def update_pose_from_odom(state: PoseState, odom_msg) -> None:
    pose = odom_msg.pose.pose
    state.x = pose.position.x
    state.y = pose.position.y
    state.z = pose.position.z
    state.qx = pose.orientation.x
    state.qy = pose.orientation.y
    state.qz = pose.orientation.z
    state.qw = pose.orientation.w


def getPosition(state: PoseState):
    """ROS-compatible replacement for Spot SDK getPosition()."""
    class _Quat:
        def __init__(self, qx, qy, qz, qw):
            self.x = qx
            self.y = qy
            self.z = qz
            self.w = qw

    return state.x, state.y, state.z, _Quat(state.qx, state.qy, state.qz, state.qw)

#!/usr/bin/env python3
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class WaitForMapNode(Node):
    def __init__(self):
        super().__init__('wait_for_map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('timeout_sec', 0.0)

        self._map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self._timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value
        self._got_map = False
        self._done = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub = self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, qos)

        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(0.2, self._check_timeout)

        self.get_logger().info(
            f"Waiting for first OccupancyGrid on '{self._map_topic}' (timeout_sec={self._timeout_sec})"
        )

    def _on_map(self, _msg: OccupancyGrid):
        if self._got_map:
            return
        self._got_map = True
        self._done = True
        self.get_logger().info("Map received. Continuing launch...")

    def _check_timeout(self):
        if self._timeout_sec <= 0.0:
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed >= self._timeout_sec:
            self.get_logger().error(
                f"Timeout waiting for map on '{self._map_topic}' after {self._timeout_sec:.1f}s"
            )
            self._done = True


def main() -> int:
    rclpy.init()
    node = WaitForMapNode()

    try:
        # Use spin_once so callbacks set the exit condition and main performs clean shutdown.
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    got_map = node._got_map
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if got_map:
        return 0
    return 1


if __name__ == '__main__':
    sys.exit(main())


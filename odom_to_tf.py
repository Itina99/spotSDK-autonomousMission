#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_bridge')
        # Sottoscrizione al topic di odometria pubblicato da Gazebo
        self.subscription = self.create_subscription(
            Odometry, '/spot/odometry', self.odom_callback, 10)
        # Strumento per pubblicare su /tf
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        t = TransformStamped()

        # Stamp temporale sincronizzato con la simulazione
        t.header.stamp = msg.header.stamp
        # Rimuoviamo eventuali slash fastidiosi dal frame iniziale ("odom_spot" invece di "/odom_spot")
        t.header.frame_id = msg.header.frame_id.lstrip('/')
        t.child_frame_id = msg.child_frame_id.lstrip('/')

        # Copia la posizione
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Copia l'orientamento
        t.transform.rotation = msg.pose.pose.orientation

        # Invia la trasformata
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

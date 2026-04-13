#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import time
import cv2
from cv_bridge import CvBridge


class HelloSpotROS(Node):
    def __init__(self):
        super().__init__('hello_spot_ros')

        # Il topic per muovere il robot. (Potrebbe essere /spot/cmd_vel a seconda del setup)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Nel tuo SDF abbiamo visto che la telecamera termica pubblica su questo topic
        self.image_sub = self.create_subscription(
            Image,
            '/spot/thermal_camera',  # Se hai altre camere, es. /spot/camera/frontleft/image
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.image_saved = False
        self.get_logger().info("🤖 Hello Spot ROS inizializzato. Avvio sequenza...")

    def image_callback(self, msg):
        """Riceve l'immagine dal topic ROS e la salva come JPEG."""
        if not self.image_saved:
            self.get_logger().info('📸 Immagine ricevuta! Salvataggio in corso...')
            try:
                # Converte il formato di ROS (sensor_msgs/Image) in un'immagine per OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

                # Salva l'immagine nella cartella corrente
                filename = 'hello_spot_ros_image.jpg'
                cv2.imwrite(filename, cv_image)

                self.image_saved = True
                self.get_logger().info(f'✅ Immagine salvata con successo: {filename}')
            except Exception as e:
                self.get_logger().error(f'Errore nel convertire o salvare l\'immagine: {e}')

    def move_robot(self, linear_x, angular_z, duration_sec):
        """Invia comandi di velocità per un tempo specifico e poi ferma il robot."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        self.get_logger().info(f'🚶 Movimento: v_x={linear_x} m/s, rot_z={angular_z} rad/s per {duration_sec}s')

        start_time = time.time()
        # Pubblica a circa 10Hz per mantenere vivo il comando
        while (time.time() - start_time) < duration_sec:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)

        # Invia un Twist vuoto (tutti zeri) per far fermare Spot
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('🛑 Robot fermo.')
        time.sleep(1.0)  # Pausa tra un'azione e l'altra


def main(args=None):
    rclpy.init(args=args)
    node = HelloSpotROS()

    # Diamo a ROS un paio di secondi per agganciare i publisher e subscriber
    time.sleep(2.0)

    # 1. "Stand twisted" e guardati intorno (Rotazione sul posto)
    node.move_robot(linear_x=0.0, angular_z=0.4, duration_sec=3.0)

    # 2. Torna in posizione (Rotazione opposta)
    node.move_robot(linear_x=0.0, angular_z=-0.4, duration_sec=3.0)

    # 3. Absolute body control (Avanza dritto)
    node.move_robot(linear_x=0.3, angular_z=0.0, duration_sec=4.0)

    # 4. Cattura l'immagine
    node.get_logger().info('👀 Aspetto di catturare un frame dalla telecamera termica...')
    wait_start = time.time()

    # Fai girare i callback di ROS per un massimo di 5 secondi per ricevere la foto
    while not node.image_saved and (time.time() - wait_start) < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.image_saved:
        node.get_logger().warn('⚠️ Non è arrivata nessuna immagine. Il topic è corretto?')

    node.get_logger().info('🏁 Sequenza completata. Spot si riposa. Terminazione in corso...')

    # Pulizia finale
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
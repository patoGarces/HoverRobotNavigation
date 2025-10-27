import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',        # topic original (desde la RPi)
            self.callback,
            10)
        self.pub = self.create_publisher(
            LaserScan,
            '/scan_relay',  # topic nuevo con timestamp corregido
            10)
        
        self.get_logger().info("iniciando...")

    def callback(self, msg):
        # Reemplazar timestamp por el reloj local
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
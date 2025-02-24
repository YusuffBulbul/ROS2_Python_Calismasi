#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# MavROS ROS2 sürümünde, "mavros_msgs" paketinin mesajları genellikle benzer isimdedir.
# State mesajına abone olmak için:
from mavros_msgs.msg import State

class TelemetrySubscriber(Node):
    def __init__(self):
        super().__init__('telemetry_subscriber')
        # MavROS'un "mavros/state" topic'ine abone oluyoruz
        self.subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        # Callback fonksiyonu için aboneliği saklıyoruz
        self.subscription

    def state_callback(self, msg):
        # Gelen State mesajını terminale yazdırıyoruz
        self.get_logger().info(f"Received state: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetrySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Node sonlandırma
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

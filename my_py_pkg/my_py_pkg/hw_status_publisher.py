#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from my_robot_interface.msg import HardwareStatus

class HardwareStatusPublisherNode(Node):  
    def __init__(self):
        super().__init__("hardware_status_publisher")

        self.publisher = self.create_publisher(HardwareStatus,"topic_name",10)

        self.timer = self.create_timer(1.0,self.sendmethod)

        self.get_logger().info("Hardware_Status_Publisher_Node has been started")

    def sendmethod(self):
        msg = HardwareStatus()
        msg.temperature = 10
        msg.are_motors_ready = True
        msg.debug_message = "İslem Basarili"
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
     main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisher(Node):  
    def __init__(self):
        super().__init__("number_publisher") 
        self.publisher = self.create_publisher(Int64,"number",10)
        self.timer = self.create_timer(1,self.sendmethod)
        self.get_logger().info("Number Publisher has been created") 

    def sendmethod(self):
        msg = Int64()
        msg.data = 5
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
     main()









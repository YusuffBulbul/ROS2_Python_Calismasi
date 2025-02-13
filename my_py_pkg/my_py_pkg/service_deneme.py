#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interface.srv import ComputeRectangleArea

class ServiceNode(Node): 
    def __init__(self):
        super().__init__("service_node")

        self.service = self.create_service(ComputeRectangleArea,"servis_name",self.callbackmethod)

        self.get_logger().info("Service has been started")

    def callbackmethod(self,request,response):
        self.get_logger().info(f"Length: {request.length} Width {request.width}")
        response.area = request.length * request.width
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
     main()
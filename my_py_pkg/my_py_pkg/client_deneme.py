#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interface.srv import ComputeRectangleArea

class ClientNode(Node):  
    def __init__(self):
        super().__init__("client_node")
        self.call_client_deneme(10.1,20.2)
        self.get_logger().info("Client has been started")

    def call_client_deneme(self,x,y):
        client = self.create_client(ComputeRectangleArea,"servis_name")
        while not client.wait_for_service(1.0):
            self.get_logger().war("waiting for service")

        request = ComputeRectangleArea.Request()
        request.length = x
        request.width = y

        future = client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self,future):
        response = future.result()

        self.get_logger().info(f"Alan: {response.area}")

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
     main()
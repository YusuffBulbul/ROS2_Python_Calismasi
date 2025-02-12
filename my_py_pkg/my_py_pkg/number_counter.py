#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from functools import partial
from example_interfaces.srv import SetBool

class NumberCounter(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.subscriber = self.create_subscription(Int64,"number",self.callbackmethod,10)
        
        self.publisher = self.create_publisher(Int64,"number_count",10)

        self.service = self.create_service(SetBool,"reset_counter",self.service_method)

        self.get_logger().info("Number Counter has been created")

        self.toplam = Int64()
        self.toplam.data = 0
        self.timer = self.create_timer(1,self.sendmethod)
        
    
    def service_method(self,request,response):
        if request.data:
            self.toplam.data = 0

        response.success = True
        response.message = "Counter reset successfully"
        return response

   
    def callbackmethod(self,msg):
        self.toplam.data += msg.data
        
    
    def sendmethod(self):

        self.publisher.publish(self.toplam)

                                               

def main():
    rclpy.init()
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import time

class RobotStatePublisherNode(Node):  
    def __init__(self):
        super().__init__("c_publisher")  

        self.publisher_ = self.create_publisher(String,"c_topic",10)
        self.timer_ = self.create_timer(0.5,self.publish_state)
        self.get_logger().info("Robot State Publisher has been started")

    def publish_state(self):
        msg = String()
        msg.data = f"{time.time()} --- C launch file çalışıyor"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
     main()
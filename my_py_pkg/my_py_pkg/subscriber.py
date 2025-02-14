#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interface.msg import Deneme


class Subscriber(Node):  
    def __init__(self):
        super().__init__("Subscriber_node")

        self.subscriber = self.create_subscription(Deneme,"topic_name",self.callbackmethod,10)

    def callbackmethod(self,msg):
        print("📩 Gelen Mesaj:")
        print(f"🕒 Sunucu Saati: {msg.sunucu_saati.saat}:{msg.sunucu_saati.dakika}:{msg.sunucu_saati.saniye}.{msg.sunucu_saati.milisaniye}")

        for i, konum in enumerate(msg.konum_bilgileri):

            print(f"\n📍 Konum {i+1}:")
            self.get_logger().info(f"   - Takım Numarası: {konum.takim_numarasi}")
            self.get_logger().info(f"   - Enlem: {konum.enlem}")
            self.get_logger().info(f"   - Boylam: {konum.boylam}")
            self.get_logger().info(f"   - İrtifa: {konum.irtifa}")
            self.get_logger().info(f"   - Dikilme: {konum.dikilme}")
            self.get_logger().info(f"   - Yönelme: {konum.yonelme}")
            self.get_logger().info(f"   - Yatış: {konum.yatis}")
            self.get_logger().info(f"   - Hız: {konum.hiz}")
            self.get_logger().info(f"   - Zaman Farkı: {konum.zaman_farki}")

           


def main(args=None):
    rclpy.init(args=args)
    node = Subscriber() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
     main()
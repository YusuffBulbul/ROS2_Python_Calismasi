#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interface.msg import Deneme
from my_robot_interface.msg import Konum
from my_robot_interface.msg import Zaman
class DenemeNode(Node):  
    def __init__(self):
        super().__init__("Deneme_Node")
        self.publisher = self.create_publisher(Deneme,"topic_name",10)
        self.timer = self.create_timer(1,self.sendmethod)

    def sendmethod(self):
        msg = Deneme()
        msg.sunucu_saati.saat = 12
        msg.sunucu_saati.dakika = 20
        msg.sunucu_saati.saniye = 50
        msg.sunucu_saati.milisaniye = 44

        msg.konum_bilgileri.append(Konum())  # 0. index
        msg.konum_bilgileri.append(Konum())  # 1. index
        msg.konum_bilgileri.append(Konum())  # 2. index

        msg.konum_bilgileri[0].takim_numarasi = 1
        msg.konum_bilgileri[0].enlem = 1.0
        msg.konum_bilgileri[0].boylam = 1.0
        msg.konum_bilgileri[0].irtifa = 1
        msg.konum_bilgileri[0].dikilme = 1.0
        msg.konum_bilgileri[0].yonelme = 1
        msg.konum_bilgileri[0].yatis = 1.0
        msg.konum_bilgileri[0].hiz = 1.0
        msg.konum_bilgileri[0].zaman_farki = 1

        msg.konum_bilgileri[1].takim_numarasi = 1
        msg.konum_bilgileri[1].enlem = 1.0
        msg.konum_bilgileri[1].boylam = 1.0
        msg.konum_bilgileri[1].irtifa = 1
        msg.konum_bilgileri[1].dikilme = 1.0
        msg.konum_bilgileri[1].yonelme = 1
        msg.konum_bilgileri[1].yatis = 1.0
        msg.konum_bilgileri[1].hiz = 1.0
        msg.konum_bilgileri[1].zaman_farki = 1


        msg.konum_bilgileri[2].takim_numarasi = 1
        msg.konum_bilgileri[2].enlem = 1.0
        msg.konum_bilgileri[2].boylam = 1.0
        msg.konum_bilgileri[2].irtifa = 1
        msg.konum_bilgileri[2].dikilme = 1.0
        msg.konum_bilgileri[2].yonelme = 1
        msg.konum_bilgileri[2].yatis = 1.0
        msg.konum_bilgileri[2].hiz = 1.0
        msg.konum_bilgileri[2].zaman_farki = 1


        self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = DenemeNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
     main()
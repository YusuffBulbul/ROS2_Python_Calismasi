#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import Telemetry

import random
from datetime import datetime

# MAVROS’tan veri almak için gerekli mesaj tipleri
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from geometry_msgs.msg import TwistStamped

# Quaternion -> Euler dönüşümü için
import tf_transformations
import math

class TelemetryUreten(Node): 
    def __init__(self):
        super().__init__("telemetry_node")
        
        # 1) MAVROS’tan gelen verileri saklayacağımız değişkenler (default değerlerle başlatıyoruz)
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        self.current_speed = 0.0
        self.current_battery = 0  # Yüzdelik (%0 - %100)
        
        # 2) MAVROS Topic’lerine abone olalım
        # GPS (enlem, boylam, irtifa)
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            10
        )
        
        # IMU (roll, pitch, yaw -> dikilme, yatış, yönelme)
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            10
        )
        
        # Velocity (hız)
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_body',
            self.velocity_callback,
            10
        )
        
        # Battery (batarya yüzdesi)
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            10
        )

        # 3) Kendi Telemetry mesajımızı yayınlamak için publisher
        self.publisher = self.create_publisher(Telemetry, "telemetry_topic", 10)

        # 4) Zamanlayıcı: Belirli aralıklarla (0.6s) mesaj yayınla
        self.timer = self.create_timer(0.6, self.sendmethod)
        
        self.get_logger().info("Telemetry Publisher has been created") 


    # -------------- MAVROS Callbacks ----------------
    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

    def imu_callback(self, msg: Imu):
        # Quaternion -> Euler (roll, pitch, yaw)
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(q)
        
        # İsterseniz rad -> derece dönüştürebilirsiniz.
        # Örnek: roll_deg = math.degrees(roll)
        # Bu örnekte direkt radyan değerleri kullanıyoruz.
        self.current_roll = roll
        self.current_pitch = pitch
        self.current_yaw = yaw

    def velocity_callback(self, msg: TwistStamped):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        # Toplam hız (3B vektör normu)
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        self.current_speed = speed

    def battery_callback(self, msg: BatteryState):
        # BatteryState.percentage 0.0 ile 1.0 arasında
        self.current_battery = int(msg.percentage * 100.0)

    # -------------- Yayınlama (Publisher) ----------------
    def sendmethod(self):
        msg = Telemetry()

        # Zaman bilgisi
        now = datetime.now()
        
        # Kilitlenme ve hedef ile ilgili alanlar rastgele üretime devam etsin
        iha_kilitlenme = random.choice([0, 1])

        # 1) MAVROS’tan aldığımız verileri mesaja doldur
        msg.enlem   = self.current_lat
        msg.boylam  = self.current_lon
        msg.irtifa  = self.current_alt

        # Dikkat: Kodunuzda “dikilme” = pitch, “yatis” = roll, “yonelme” = yaw gibi eşledik
        msg.dikilme = self.current_pitch
        msg.yatis   = self.current_roll
        msg.yonelme = math.degrees(self.current_yaw)  # Dereceye çevirmek isterseniz
        # msg.yonelme = self.current_yaw  # Radyan olarak da bırakabilirsiniz

        msg.hiz = round(self.current_speed, 2)
        msg.batarya = self.current_battery

        # 2) Diğer alanlar rastgele veya sabit
        msg.takim_numarasi = 1
        msg.otonom = random.choice([0, 1])
        msg.kilitlenme = iha_kilitlenme

        if iha_kilitlenme == 1:           
            msg.hedef_merkez_x = random.randint(100, 500)
            msg.hedef_merkez_y = random.randint(100, 500)
            msg.hedef_genislik = random.randint(10, 50)
            msg.hedef_yukseklik = random.randint(10, 50)
        else:
            msg.hedef_merkez_x = 0
            msg.hedef_merkez_y = 0
            msg.hedef_genislik = 0
            msg.hedef_yukseklik = 0

        # Zaman alanları
        msg.saat = now.hour
        msg.dakika = now.minute
        msg.saniye = now.second
        msg.milisaniye = now.microsecond // 1000

        # 3) Mesajı yayınla
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = TelemetryUreten() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


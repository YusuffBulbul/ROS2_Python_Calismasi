import subprocess
import rclpy
from rclpy.node import Node
from my_robot_interface.srv import LaunchFile  # Kendi servis tanımınıza uyarlayın

class LaunchManager(Node):
    def __init__(self):
        super().__init__('launch_manager')
        self.current_process = None    # A, B, C vb. durum launch dosyaları için
        self.always_process = None     # her_zaman.launch.xml için

        # Başlangıçta "her_zaman.launch.xml" dosyasını çalıştır
        self.start_always_launch()

        # "launch_file" adında servis sunucusu oluşturuyoruz.
        self.srv = self.create_service(LaunchFile, 'launch_file', self.launch_file_callback)
        self.get_logger().info("Launch Manager servisi başlatıldı.")

    def start_always_launch(self):
        """Uygulama başladığında her zaman çalışacak launch dosyasını başlatır."""
        command = ['ros2', 'launch', 'my_robot_bringup', 'her_zaman.launch.xml']
        self.always_process = subprocess.Popen(command)
        self.get_logger().info("her_zaman.launch.xml başlatıldı ve sürekli çalışacak.")

    def launch_file_callback(self, request, response):
        launch_file_name = request.launch_file
        self.get_logger().info(f"Gelen istek: {launch_file_name}")

        # Eğer mevcut bir ikinci launch süreci varsa (A/B/C vb.), önce onu kapat
        if self.current_process is not None:
            self.get_logger().info('Mevcut ikinci launch süreci sonlandırılıyor...')
            self.current_process.terminate()  # SIGTERM gönderir
            self.current_process.wait()       # Sürecin tamamen kapanmasını bekleyin
            self.current_process = None

        # "stop" komutu alındıysa, sadece mevcut süreci sonlandırmış olduk ve geri dönüyoruz.
        # (her_zaman.launch.xml kapatılmaz)
        if launch_file_name.lower() == "stop":
            response.success = True
            response.message = "Mevcut ikinci launch süreci durduruldu. 'her_zaman' ise çalışmaya devam ediyor."
            return response

        # Yeni launch dosyasını (A, B, C, vb.) başlatıyoruz. "her_zaman" zaten çalışmaya devam ediyor.
        command = ['ros2', 'launch', 'my_robot_bringup', launch_file_name]
        self.get_logger().info(f"{launch_file_name} başlatılıyor (her_zaman da çalışıyor)...")
        try:
            self.current_process = subprocess.Popen(command)
            response.success = True
            response.message = f"{launch_file_name} başarıyla başlatıldı (her_zaman çalışmaya devam ediyor)."
        except Exception as e:
            response.success = False
            response.message = f"Hata: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    manager = LaunchManager()
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.get_logger().info("Kapatılıyor...")
    finally:
        # Program tamamen kapatılırken her iki süreci de durdurun.
        if manager.current_process is not None:
            manager.current_process.terminate()
            manager.current_process.wait()
        if manager.always_process is not None:
            manager.always_process.terminate()
            manager.always_process.wait()

        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

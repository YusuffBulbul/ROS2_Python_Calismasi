import subprocess
import rclpy
from rclpy.node import Node
from my_robot_interface.srv import LaunchFile  

class LaunchManager(Node):
    def __init__(self):
        super().__init__('launch_manager')
        self.current_process = None

        # "launch_file" adında servis sunucusu oluşturuyoruz.
        self.srv = self.create_service(LaunchFile, 'launch_file', self.launch_file_callback)
        self.get_logger().info("Launch Manager servisi başlatıldı.")

    def launch_file_callback(self, request, response):
        launch_file_name = request.launch_file
        self.get_logger().info(f"Gelen istek: {launch_file_name}")

        # Eğer çalışan bir launch süreci varsa, kapatın.
        if self.current_process is not None:
            self.get_logger().info('Mevcut launch süreci sonlandırılıyor...')
            self.current_process.terminate()  # SIGTERM gönderir.
            self.current_process.wait()       # Sürecin tamamen kapanmasını bekleyin.
            self.current_process = None

        # "stop" komutu alındıysa, sadece mevcut süreci sonlandırıp geri dönün.
        if launch_file_name.lower() == "stop":
            response.success = True
            response.message = "Mevcut launch süreci durduruldu."
            return response

        # Yeni XML formatındaki launch dosyasını başlatın.
        # "package_name" kısmını, ilgili launch dosyalarınızın bulunduğu paketin adı ile değiştirin.
        command = ['ros2', 'launch', 'my_robot_bringup', launch_file_name]
        self.get_logger().info(f"{launch_file_name} başlatılıyor...")
        try:
            self.current_process = subprocess.Popen(command)
            response.success = True
            response.message = f"{launch_file_name} başarıyla başlatıldı."
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
        if manager.current_process is not None:
            manager.current_process.terminate()
            manager.current_process.wait()
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

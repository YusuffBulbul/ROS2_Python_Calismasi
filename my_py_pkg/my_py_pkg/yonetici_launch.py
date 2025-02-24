import subprocess
import rclpy
from rclpy.node import Node
from my_robot_interface.srv import LaunchFile  

class LaunchManager(Node):
    def __init__(self):
        super().__init__('launch_manager')
        self.current_process = None    
        self.always_process = None     

        self.start_always_launch()


        self.srv = self.create_service(LaunchFile, 'launch_file', self.launch_file_callback)
        self.get_logger().info("Launch Manager servisi başlatıldı.")

    def start_always_launch(self):

        command = ['ros2', 'launch', 'my_robot_bringup', 'her_zaman.launch.xml']
        self.always_process = subprocess.Popen(command)
        self.get_logger().info("her_zaman.launch.xml başlatıldı ve sürekli çalışacak.")

    def launch_file_callback(self, request, response):
        launch_file_name = request.launch_file
        self.get_logger().info(f"Gelen istek: {launch_file_name}")

        if self.current_process is not None:
            self.get_logger().info('Mevcut ikinci launch süreci sonlandırılıyor...')
            self.current_process.terminate()  
            self.current_process.wait()       
            self.current_process = None


        if launch_file_name.lower() == "stop":
            response.success = True
            response.message = "Mevcut ikinci launch süreci durduruldu. 'her_zaman' ise çalışmaya devam ediyor."
            return response


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

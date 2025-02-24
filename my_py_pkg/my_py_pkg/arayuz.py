import threading
import tkinter as tk
import rclpy
from rclpy.node import Node
from my_robot_interface.srv import LaunchFile  

class ROSClient(Node):
    def __init__(self):
        super().__init__('launch_client')
        self.cli = self.create_client(LaunchFile, 'launch_file')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servis bulunamadı, tekrar deneniyor...')
        self.get_logger().info("Servis bağlantısı sağlandı.")

    def send_request(self, launch_file_name: str):
        req = LaunchFile.Request()
        req.launch_file = launch_file_name
        future = self.cli.call_async(req)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().error(response.message)
        except Exception as e:
            self.get_logger().error(f"Hata: {str(e)}")

def ros_spin(client):
    rclpy.spin(client)

def main():
    rclpy.init()
    ros_client = ROSClient()


    ros_thread = threading.Thread(target=ros_spin, args=(ros_client,), daemon=True) # Ayrı bır threadde çalıştırmalıyız GUI den block yememek için
    ros_thread.start()


    root = tk.Tk()
    root.title("ROS2 Launch Kontrol")


    frame = tk.Frame(root)
    frame.pack(pady=20)

    btnA = tk.Button(frame, text="A Durumu", width=15,
                     command=lambda: ros_client.send_request("A.launch.xml"))
    btnB = tk.Button(frame, text="B Durumu", width=15,
                     command=lambda: ros_client.send_request("B.launch.xml"))
    btnC = tk.Button(frame, text="C Durumu", width=15,
                     command=lambda: ros_client.send_request("C.launch.xml"))
    
    btnA.grid(row=0, column=0, padx=10)
    btnB.grid(row=0, column=1, padx=10)
    btnC.grid(row=0, column=2, padx=10)


    stop_btn = tk.Button(root, text="Durdur", width=20,
                         command=lambda: ros_client.send_request("stop"))
    stop_btn.pack(pady=10)


    root.mainloop()


    ros_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
from datetime import datetime


class Listener(Node):
    def __init__(self):
        super().__init__("real_listener_1")

        self.sub_servo = self.create_subscription(
            msg_type=String,
            topic="data_servo",
            callback=self.callback_servo,
            qos_profile=10
        )
        self.sub_sensor = self.create_subscription(
            msg_type=String,
            topic="sensor_value",
            callback=self.callback_sensor,
            qos_profile=10
        )
        self.sub_menu = self.create_subscription(
            msg_type=String,
            topic="menu",
            callback=self.callback_menu,
            qos_profile=10
        )
        

        # Membuat nama file CSV berdasarkan waktu sekarang
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_file_name = f"Sensor/sensor_data_{current_time}.csv"

        # Membuka file CSV untuk ditulis
        self.csv_file = open(self.csv_file_name, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.menu_choice = 0

        self.sensor_buffer = []
        self.another_buffer = []

    def callback_menu(self, msg):
        self.menu_choice = msg.data
        self.get_logger().info(f'Received menu choice: {self.menu_choice}')

            

    def callback_sensor(self, msg):
        sensor_data = msg.data
        data_list = sensor_data.split(',')
        self.sensor_buffer.append(data_list)

        # Cek apakah kedua buffer sudah siap untuk diproses
        if self.is_ready_to_process():
            self.process_data()


    def callback_servo(self, msg):
        sensor_data = msg.data
        another_data_list = sensor_data.split(',')

        if another_data_list[-1] == '':  # Periksa apakah nilai terakhir kosong
            another_data_list.pop()  # Hapus nilai terakhir kosong jika ada

        # Tambahkan nilai terakhir dari another_data ke another_data_list
        if another_data_list:
            last_value = another_data_list[0]
            another_data_list.append(last_value)

        self.another_buffer.append(another_data_list)


        # Cek apakah kedua buffer sudah siap untuk diproses
        if self.is_ready_to_process():
            self.process_data()
    def is_ready_to_process(self):
        # Tentukan kondisi kapan kedua buffer sudah siap untuk diproses
        return len(self.another_buffer) > 0  # Misalnya, proses ketika buffer another sudah terisi
    
    def process_data(self):
        # Lakukan pemrosesan data dari kedua buffer sesuai dengan kebutuhan
        if self.another_buffer and self.sensor_buffer:
            combined_data = self.another_buffer.pop(0) + self.sensor_buffer.pop(0) # Contoh penggabungan data
        if self.menu_choice == '3':
            self.csv_writer.writerow(combined_data)
        
    def close(self):
        # Menutup file CSV
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        # Menutup node dan file CSV saat program berhenti
        listener.close()
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
from datetime import datetime


class Listener(Node):
    def __init__(self):
        super().__init__("real_listener_1")

        self.sub = self.create_subscription(
            msg_type=String,
            topic="sensor_value",
            callback=self.callback,
            qos_profile=10
        )

        # Membuat nama file CSV berdasarkan waktu sekarang
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_file_name = f"Sensor/sensor_data_{current_time}.csv"

        # Membuka file CSV untuk ditulis
        self.csv_file = open(self.csv_file_name, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

    def callback(self, msg):
        sensor_data = msg.data
        self.get_logger().info(
            '\n\t Sensor: {}\n'.format(
                sensor_data
            )
        )
        data_list = sensor_data.split(',')
        self.csv_writer.writerow([x for x in data_list])

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

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
from datetime import datetime
from threading import Lock


class Listener(Node):
    def __init__(self):
        super().__init__("Sensor")

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
        self.sub_gripper = self.create_subscription(
            msg_type=String,
            topic="Gripper",
            callback=self.callback_gripper,
            qos_profile=10
        )
        self.csv_file_name = ""
        self.csv_file = None
        self.csv_writer = None
        self.menu_choice = "0"
        self.condition_gripper = "0"
        self.sensor_buffer = []
        self.another_buffer = None

    def callback_gripper(self, msg):
        self.condition_gripper = msg.data


    def callback_menu(self, msg):
        self.menu_choice = msg.data
        if self.menu_choice == '3':
            current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.csv_file_name = f"Sensor/sensor_data_{current_time}.csv"
            self.csv_file = open(self.csv_file_name, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            

    def callback_sensor(self, msg):
        sensor_data = msg.data
        self.data_list = sensor_data.split(',')
        
        self.process_data()

    def callback_servo(self, msg):
        servo_data = msg.data
        self.another_buffer = servo_data.split(',')
    
        self.process_data()
        
    def process_data(self):
        if self.menu_choice == '3' and self.condition_gripper != '0':
            combined_sensor_data = self.data_list
            combined_data = self.another_buffer + combined_sensor_data if self.another_buffer else combined_sensor_data
            print(combined_data)
            self.csv_writer.writerow(combined_data)
        # elif self.menu_choice == '10' and self.condition_gripper != '0':
        #     combined_sensor_data = self.data_list
        #     combined_data = self.another_buffer + combined_sensor_data if self.another_buffer else combined_sensor_data
        #     print(combined_data)
        #     self.csv_writer.writerow(combined_data)
        
    def close(self):
        if self.csv_file:
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

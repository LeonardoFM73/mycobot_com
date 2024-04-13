import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Sender(Node):
    def __init__(self):
        super().__init__("Gripper")

    def start(self):
        pub = self.create_publisher(
            msg_type=Int32,
            topic="menu",
            qos_profile=10
        )
        rate = self.create_rate(30)  # 30hz

        while rclpy.ok():
            print("Pilihan Menu:")
            print("1. Diam")
            print("2. Kirim nilai sensor ke ROS")
            print("3. Kirim posisi servo ke ROS")

            # Minta pengguna untuk memilih menu (1, 2, atau 3)
            menu_choice = input("Masukkan pilihan (1/2/3): ")

            try:
                menu_choice = int(menu_choice)
                if menu_choice < 1 or menu_choice > 3:
                    Node.get_logger().warning("Pilihan tidak valid. Masukkan angka 1, 2, atau 3.", skip_first=True)
                    continue

                # Buat pesan untuk dikirim ke Arduino
                msg = Int32()
                msg.data = menu_choice

                # Kirim pesan ke topik 'menu_choice'
                pub.publish(msg)
                rate.sleep()

            except Exception as e:
                print(e)

def main(args=None):
    rclpy.init(args=args)
    
    talker = Sender()
    talker.start()
    rclpy.spin(talker)
    
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
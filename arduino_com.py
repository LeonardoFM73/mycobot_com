import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Sender(Node):
    def __init__(self):
        super().__init__("Gripper")

    def start(self):
        self.sub = self.create_subscription(
            msg_type=String,
            topic="menu",
            callback=self.callback,
            qos_profile=10
        )
        pub = self.create_publisher(
            msg_type=String,
            topic="servo_value",
            qos_profile=10
        )
        rate = self.create_rate(30)  # 30hz

        while rclpy.ok():
            print("Pilihan Menu:")
            print("1. Buka")
            print("2. Tutup")
            print("3. Collect Data")

            # Minta pengguna untuk memilih menu (1, 2, atau 3)
            menu_choice = input("Masukkan pilihan (1/2/3): ")

            try:
                menu_choice = str(menu_choice)
                
                # Buat pesan untuk dikirim ke Arduino
                msg = String()
                msg.data = menu_choice

                # Kirim pesan ke topik 'menu_choice'
                pub.publish(msg)
                # rate.sleep()

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
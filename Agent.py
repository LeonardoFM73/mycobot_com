import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Sender(Node):
    def __init__(self):
        super().__init__('Gripper')

        self.pub_menu = self.create_publisher(
            msg_type=String,
            topic='menu',
            qos_profile=10
        )

    def send_command(self, command):
        msg = String()
        msg.data = str(command)
        self.pub_menu.publish(msg)
        self.get_logger().info(f'Sending command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    sender = Sender()

    try:
        while rclpy.ok():
            print("Pilihan Menu:")
            print("1. Buka")
            print("2. Tutup")
            print("3. Collect Data")

            menu_choice = input("Masukkan pilihan (1/2/3): ")

            if menu_choice in ['1', '2', '3']:
                sender.send_command(menu_choice)
            else:
                print("Pilihan tidak valid. Masukkan angka 1, 2, atau 3.")

    except KeyboardInterrupt:
        pass

    sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

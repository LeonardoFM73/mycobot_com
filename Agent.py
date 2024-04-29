import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

cond = False
class Sender(Node):
    def __init__(self):
        super().__init__('Gripper')

        self.pub_menu = self.create_publisher(
            msg_type=String,
            topic='menu',
            qos_profile=10
        )

        self.pub_mycobot = self.create_publisher(
            msg_type=JointState,
            topic="koordinat",
            qos_profile=10
        )
        self.sub_mycobot = self.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self.callback_coord,
            qos_profile=10,
        )
        self.sub_servo = self.create_subscription(
            msg_type=String,
            topic="Status_collect",
            callback=self.callback_servo,
            qos_profile=10,
        )


        # pub joint state
        self.joint_state_send = JointState()
        self.joint_state_send.header = Header()

        self.joint_state_send.name = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]

        self.joint_state_send.velocity = [0.0, ]
        self.joint_state_send.effort = []

    def send_robot(self,coordinate):
            self.joint_state_send.header.stamp = self.get_clock().now().to_msg()
            self.joint_state_send.position = coordinate
            # self.joint_state_send.velocity = 50
            self.pub_mycobot.publish(self.joint_state_send)

    def send_command(self, command):
        msg = String()
        msg.data = str(command)
        self.pub_menu.publish(msg)
        self.get_logger().info(f'Sending command: {msg.data}')

    def callback_coord(self,msg):
        global coord_data
        coord_data = list(msg.position)
    def callback_servo(self,msg):
        global cond  # Gunakan variabel global cond
        self.status = msg.data
        # self.get_logger().info(f'Sending command: {self.status}')
        if self.status == '6':
            cond=True


def main(args=None):
    global cond  # Gunakan variabel global con
    rclpy.init(args=args)
    sender = Sender()

    try:
        while rclpy.ok():

            print("Pilihan Menu:")
            print("1. Buka")
            print("2. Tutup")
            print("3. Collect Data")

            menu_choice = input("Masukkan pilihan (1/2/3): ")

            if menu_choice in ['1', '2']:
                sender.send_command(menu_choice)
            elif menu_choice in ['3']:
                res = [float(-78.9),float(-188.5),float(283.2),float(-87.1),float(1.21),float(179.62)]
                sender.send_robot(res)
                time.sleep(5)
                sender.send_command(menu_choice)
                while not cond:
                    rclpy.spin_once(sender)
                    print(cond)
                    # time.sleep(5) # input dari sensor
                time.sleep(3)
                res1 = [float(37.1),float(-161.3),float(224.0),float(-82.62),float(0.67),float(-142.35)]
                sender.send_robot(res1)
                
            else:
                print("Pilihan tidak valid. Masukkan angka 1, 2, atau 3.")

    except KeyboardInterrupt:
        pass

    sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

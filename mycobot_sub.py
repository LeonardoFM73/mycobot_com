import math

import rclpy
from pymycobot.mycobot import MyCobot
# from pymycobot.mycobotsocket import MyCobotSocket
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class Sender(Node):
    def __init__(self):
        super().__init__("real_listener")
        
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)
   
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MyCobot(port,str(baud))

        self.sub = self.create_subscription(
            msg_type=JointState,
            topic="koordinat",
            callback=self.callback_coord,
            qos_profile=10
        )

    def callback_coord(self,msg):
        self.angles_data = [list(msg.position)]


    def gerak_angles(self):
        # 获取joint输入的数据，发送给机械臂
        j_value = []
        for i in self.angles_data:
            j_value.append(float(i))
            
        self.speed = 50
        
        res = [j_value, self.speed]

        try:
            self.mc.send_angles(*res)
        except Exception as e:
            pass
        self.show_j_date(j_value)

def main(args=None):
    rclpy.init(args=args)

    Sender = Sender()

    rclpy.spin(Sender)

    Sender.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


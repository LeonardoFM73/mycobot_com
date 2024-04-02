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
    def terima_koord(self):
        return 0
    def gerak_joint(self):
        # 获取 coord 输入的数据，发送给机械臂
        c_value = []
        for i in self.all_c:
            c_value.append(float(i.get()))
        self.speed = (
            int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        )
        
        try:
            self.mc.send_coords(c_value,self.speed, self.model)
        except Exception as e:
            pass
        self.show_j_date(c_value, "coord")

    def gerak_angles(self):
        # 获取joint输入的数据，发送给机械臂
        j_value = []
        for i in self.all_j:
            j_value.append(float(i.get()))
            
        self.speed = (
            int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        )
        
        res = [j_value, self.speed]

        try:
            self.mc.send_angles(*res)
        except Exception as e:
            pass
        self.show_j_date(j_value)

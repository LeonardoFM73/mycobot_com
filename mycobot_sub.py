import math

import rclpy
from pymycobot.mycobot import MyCobot
# from pymycobot.mycobotsocket import MyCobotSocket
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class Sender(Node):
    def __init__(self):
        super().__init__("Mycobot")
        
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
        self.pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )
        self.timer = self.create_timer(0.1,self.publish_joint_states)


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
        self.speed = 50
    
    def publish_joint_states(self):
        res1 = self.mc.get_coords()
        self.get_logger().info("res: {}".format(res1))
        # self.get_logger().info("res: {}".format(self.speed))

        # publish angles.
        self.joint_state_send.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_send.position = res1
        self.pub.publish(self.joint_state_send)

    def release_all_servos(self):
        self.mc.release_all_servos()
    
    def callback_coord(self,msg):
        self.coord_data = list(msg.position)
        self.velocity = list(msg.velocity)
        
        j_value = self.coord_data
        print(self.coord_data)
        # for i in self.coord_data[0]:
        #     j_value.append(i)
            
        self.speed = self.velocity[0]
     

        self.mc.sync_send_coords(j_value, int(self.speed),1
                                 )
            
        
        # self.mc.sync_send_angles(j_value, self.speed)
        # self.show_j_date(j_value)

        # def send_input(self,dates


def main(args=None):
    rclpy.init(args=args)

    Gerak = Sender()
    

    try:
        rclpy.spin(Gerak)
    except KeyboardInterrupt:
        Gerak.destroy_node()
        rclpy.shutdown()
        Gerak.release_all_servos()


if __name__ == "__main__":
    main()

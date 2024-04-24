import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class Talker(Node):

    def __init__(self):
        super().__init__("Publish_coord")

        pub = self.create_publisher(
            msg_type=JointState,
            topic="koordinat",
            qos_profile=10
        )
        rate = self.create_rate(30)  # 30hz

        # pub joint state
        joint_state_send = JointState()
        joint_state_send.header = Header()

        joint_state_send.name = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]

        joint_state_send.velocity = [0.0, ]
        joint_state_send.effort = []
        
        while rclpy.ok():
            
            rclpy.spin_once(self)
            # get real angles from server.
            res = [float(-78.9),float(-188.5),float(283.2),float(-87.1),float(1.21),float(179.62)]

            try:
                if res[0] == res[1] == res[2] == 0.0:
                    continue
                radians_list = [
                    res[0],
                    res[1],
                    res[2],
                    res[3],
                    res[4],
                    res[5],
                ]
                self.get_logger().info("res: {}".format(radians_list))

                # publish angles.
                joint_state_send.header.stamp = self.get_clock().now().to_msg()
                joint_state_send.position = radians_list
                pub.publish(joint_state_send)
                rate.sleep()
            except Exception as e:
                print(e)
            
            


def main(args=None):
    rclpy.init(args=args)
    
    talker = Talker()
    talker.start()
    rclpy.spin(talker)
    
    talker.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()
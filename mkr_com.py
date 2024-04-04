import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class Listener(Node):
    def __init__(self):
        super().__init__("real_listener_1")

        self.sub = self.create_subscription(
            msg_type=String,
            topic="sensor_value",
            callback=self.callback,
            qos_profile=10
        )

    def callback(self, msg):
        sensor_data = msg.data
        self.get_logger().info(
            '\n\t Sensor: {}\n'.format(
                sensor_data
            )
        )


def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    rclpy.spin(listener)

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

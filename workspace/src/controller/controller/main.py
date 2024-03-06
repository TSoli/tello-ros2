import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty


class Controller(Node):
    """
    Sends control instructions to the Tello drone.
    """

    def __init__(self) -> None:
        super().__init__("controller")

        self._control_publisher = self.create_publisher(Twist, "control", 10)
        self._takeoff_publisher = self.create_publisher(Empty, "takeoff", 10)
        self._land_publisher = self.create_publisher(Empty, "land", 10)

        self._timer = self.create_timer(8, self.send_control_msg)
        self.num_actions = 0

    def send_control_msg(self) -> None:
        if self.num_actions == 0:
            self.takeoff()
        elif self.num_actions == 1:
            # move forward?
            pass
            self.send_twist(0.0, 20.0, 0.0, 0.0)
        elif self.num_actions == 2:
            # rotate left?
            pass
            self.send_twist(0.0, 0.0, 0.0, 30.0)
        elif self.num_actions == 3:
            self.land()
        else:
            self.get_logger().info("Done")
            raise SystemExit

        self.num_actions += 1

    def takeoff(self) -> None:
        self._takeoff_publisher.publish(Empty())

    def land(self) -> None:
        self._land_publisher.publish(Empty())

    def send_twist(self, x: float, y: float, z: float, rot: float) -> None:
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = rot
        self._control_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

    try:
        rclpy.spin(controller)
    except SystemExit:
        pass

    controller.destroy_node()
    rclpy.shutdown()

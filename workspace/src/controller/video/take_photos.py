import os
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg


class TakePhotos(Node):
    def __init__(self):
        super().__init__("take_photos")
        self.declare_parameter("save_dir", "images")
        self.save_dir = (
            self.get_parameter("save_dir").get_parameter_value().string_value
        )

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            ImageMsg, "camera/image", self.show_image, 10
        )

    def show_image(self, msg: ImageMsg) -> None:
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("frame", img)

        key = cv2.waitKey(0) & 0xFF

        if key == ord("s"):
            now = datetime.now().strftime("%Y%m%d-%H%M%S")
            cv2.imwrite(os.path.join(self.save_dir, f"{now}.jpg"), img)
        elif key == ord("q"):
            cv2.destroyAllWindows()
            raise SystemExit


def main() -> None:
    rclpy.init()
    rclpy.spin(TakePhotos())

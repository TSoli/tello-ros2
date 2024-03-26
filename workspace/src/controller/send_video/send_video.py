import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs import msg


class SendVideo(Node):
    def __init__(self) -> None:
        super().__init__("send_video")

        self.declare_parameter("filename", "")
        self.cv_bridge = CvBridge()
        self.image_pub = self.create_publisher(msg.Image, "camera", 10)
        self.video = self._get_video()

        fps = self.video.get(cv2.CAP_PROP_FPS)
        self.create_timer(1 / fps, self.send_frame)

    def send_frame(self) -> None:
        if not self.video.isOpened():
            self.video.release()
            raise SystemExit

        ret, frame = self.video.read()

        if not ret:
            self.video.release()
            raise SystemExit

        self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))

    def _get_video(self) -> cv2.VideoCapture:
        filename = self.get_parameter("filename").get_parameter_value().string_value
        assert os.path.exists(filename), f"file: {filename} does not exist."

        cap = cv2.VideoCapture(filename)
        assert cap.isOpened(), f"Could not open {filename}."

        return cap


def main() -> None:
    rclpy.init()
    rclpy.spin(SendVideo())


if __name__ == "__main__":
    main()

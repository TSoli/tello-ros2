import os
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header as HeaderMsg


class SendVideo(Node):
    def __init__(self) -> None:
        super().__init__("send_video")

        self.declare_parameter("filename", "")
        self.cv_bridge = CvBridge()
        self.image_pub = self.create_publisher(ImageMsg, "camera", 10)
        self.video = self._get_video()
        self.start_time = None

        fps = self.video.get(cv2.CAP_PROP_FPS)
        self.create_timer(1 / fps, self.send_frame)

    def send_frame(self) -> None:
        """Publish a frame from the video"""
        if not self.video.isOpened():
            self.video.release()
            raise SystemExit

        ret, frame = self.video.read()

        if not ret:
            self.video.release()
            raise SystemExit

        header = HeaderMsg()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera"
        msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8", header)

        self.image_pub.publish(msg)

        if self.start_time is None:
            self.start_time = header.stamp

            filename = self.get_parameter("filename").get_parameter_value().string_value
            filename = Path(filename).stem
            with open(f"{filename}_start_time.yml", "w") as f:
                f.write(f"sec: {self.start_time.sec}\n")
                f.write(f"nanosec: {self.start_time.nanosec}")

    def _get_video(self) -> cv2.VideoCapture:
        """Open the video file"""
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

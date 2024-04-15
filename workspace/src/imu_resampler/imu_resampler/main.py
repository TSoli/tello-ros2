__author__ = "Tariq Soliman"
__email__ = "t.soliman@uqconnect.edu.au"

import rclpy
from rclpy.clock import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Header as HeaderMsg
from transforms3d.euler import euler2quat, quat2euler

NANO = 1e-9


class ImuResampler(Node):
    """Upsample imu readings"""

    def __init__(self) -> None:
        super().__init__("imu_resampler")
        self._prev_imu_readings = {}
        self._gradients = {}

        self._imu_sub = self.create_subscription(ImuMsg, "imu", self.receive_imu, 10)
        self._imu_resampled_pub = self.create_publisher(ImuMsg, "imu_resampled", 10)
        self.create_timer(0.005, self.resample_imu)

    def receive_imu(self, msg: ImuMsg) -> None:
        """Update IMU readings."""
        curr_msg_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * NANO)
        dt = curr_msg_time - self._prev_imu_readings.get("t", curr_msg_time - 1)
        ax, ay, az = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        rx, ry, rz = quat2euler(
            (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        )

        # These just give 0 for the first reading
        self._gradients["ax"] = (ax - self._prev_imu_readings.get("ax", ax)) / dt
        self._gradients["ay"] = (ay - self._prev_imu_readings.get("ay", ay)) / dt
        self._gradients["az"] = (az - self._prev_imu_readings.get("az", az)) / dt
        self._gradients["gx"] = (rx - self._prev_imu_readings.get("rx", rx)) / dt
        self._gradients["gy"] = (ry - self._prev_imu_readings.get("ry", ry)) / dt
        self._gradients["gz"] = (rz - self._prev_imu_readings.get("rz", rz)) / dt

        self._prev_imu_readings["t"] = curr_msg_time
        self._prev_imu_readings["ax"] = ax
        self._prev_imu_readings["ay"] = ay
        self._prev_imu_readings["az"] = az
        self._prev_imu_readings["rx"] = rx
        self._prev_imu_readings["ry"] = ry
        self._prev_imu_readings["rz"] = rz
        self._prev_imu_readings["frame_id"] = msg.header.frame_id

    def resample_imu(self) -> None:
        """Send the resampled IMU messages."""
        if not self._prev_imu_readings:
            return

        t2 = self.get_clock().now()
        sec, nano = t2.seconds_nanoseconds()
        t2_sec = sec + (nano * NANO)
        dt = t2_sec - self._prev_imu_readings["t"]

        msg = ImuMsg()
        # TODO: This number came from calibration of Tello with Kalibr
        # it would be better to read it in instead
        msg.header.stamp = (t2 + Duration(nanoseconds=101_314_505)).to_msg()

        msg.header.frame_id = self._prev_imu_readings["frame_id"]

        rx = self._prev_imu_readings["rx"] + (self._gradients["gx"] * dt)
        ry = self._prev_imu_readings["ry"] + (self._gradients["gy"] * dt)
        rz = self._prev_imu_readings["rz"] + (self._gradients["gz"] * dt)

        qw, qx, qy, qz = euler2quat(rx, ry, rz)

        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz

        msg.linear_acceleration.x = self._prev_imu_readings["ax"] + (
            self._gradients["ax"] * dt
        )
        msg.linear_acceleration.y = self._prev_imu_readings["ay"] + (
            self._gradients["ay"] * dt
        )
        msg.linear_acceleration.z = self._prev_imu_readings["az"] + (
            self._gradients["az"] * dt
        )

        msg.angular_velocity.x = self._gradients["gx"]
        msg.angular_velocity.y = self._gradients["gy"]
        msg.angular_velocity.z = self._gradients["gz"]

        self._imu_resampled_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    imu_resampler = ImuResampler()
    rclpy.spin(imu_resampler)
    rclpy.shutdown()

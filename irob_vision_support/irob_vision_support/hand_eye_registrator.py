"""Utilities for hand-eye registration using a Cartucho et al. (2021) cylindrical marker"""

import os
import math
from os.path import isfile
from typing import Tuple
import time
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
import rclpy
import rclpy.time
import rclpy.timer
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from std_msgs.msg import Header
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from scipy.spatial.distance import euclidean
from scipy.spatial.transform import Slerp
from irob_utils.rigid_transform_3D import rigid_transform_3D
from irob_utils.conversions import arr_to_pose, pose_to_arr


class HandEyeRegistrator(Node):
    """Coordinates the registration process, and finds the transform between camera and joint spaces"""

    def __init__(self):
        super().__init__("hand_eye_registrator")

        self.registration_id = self.get_parameter("registration_id").get_parameter_value().string_value
        self.arm = self.get_parameter("arm").get_parameter_value().string_value
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.save_cylmarker_poses = self.get_parameter("save_cylmarker_poses").get_parameter_value().bool_value
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.velocity = self.get_parameter("velocity").get_parameter_value().double_value
        self.registration_dir_path = self.get_parameter("registration_dir_path").get_parameter_value().string_value
        self.poses_dir_path = self.get_parameter("poses_dir_path").get_parameter_value().string_value
        self.auto_reg_file_path = self.get_parameter("auto_reg_file_path").get_parameter_value().string_value

        self.rate = self.create_rate(self.frequency)
        # We treat poses as 7-long 1D arrays until publication, in this order:
        # position x, y, z; rotation x, y, z, w
        self.auto_registration_poses = np.zeros((0, 7))
        self.gathered_robot_poses = np.zeros((0, 7))
        self.gathered_cylmarker_poses = np.zeros((0, 7))
        self.cylmarker_tf: PoseStamped = None
        self.measured_cp: PoseStamped = None
        self.measured_jaw: JointState = None
        self.clutch_N = 0

        self.cylmarker_tf_sub = self.create_subscription(
            PoseStamped,
            "cylmarker_tf",
            self.cb_cylmarker_tf,
            10
        )
        self.measured_cp_sub = self.create_subscription(
            PoseStamped,
            f"/{self.arm}/measured_cp",
            self.cb_measured_cp,
            10
        )
        self.jaw_measured_js_sub = self.create_subscription(
            JointState,
            f"/{self.arm}/jaw/measured_js",
            self.cb_jaw_measured_js,
            10
        )
        self.manip_clutch_sub = self.create_subscription(
            Joy,
            f"/{self.arm}/manip_clutch",
            self.cb_manip_clutch,
            10
        )

        self.servo_cp_pub = self.create_publisher(
            PoseStamped,
            f"/{self.arm}/servo_cp",
            10
        )
        self.servo_jaw_pub = self.create_publisher(
            JointState,
            f"/{self.arm}/jaw/servo_jp",
            10
        )

    def cb_cylmarker_tf(self, msg: PoseStamped):
        """Callback function for cylmarker pose."""
        self.cylmarker_tf = msg

    def cb_measured_cp(self, msg: PoseStamped):
        """Callback function for measured_cp."""
        self.measured_cp = msg

    def cb_jaw_measured_js(self, msg: JointState):
        """Callback function jaw/measured_js"""
        self.measured_jaw = msg

    def cb_manip_clutch(self, msg: Joy):
        """Callback function when clutch button is pressed.
        Collect one sample from the positions.
        """
        if self.cylmarker_tf is not None and self.clutch_N > 0 and msg.buttons[0] == 0:
            self.gather_actual_position()

        self.clutch_N += 1

    def gather_actual_position(self):
        """Gather a single position from the camera and the robot."""
        if self.cylmarker_tf.header.frame_id != "invalid":
            self.gathered_robot_poses = np.vstack(
                (self.gathered_robot_poses, pose_to_arr(self.measured_cp.pose))
            )
            self.gathered_cylmarker_poses = np.vstack(
                (self.gathered_cylmarker_poses, pose_to_arr(self.cylmarker_tf.pose))
            )

            self.get_logger().info(
                f"Poses collected: {self.gathered_robot_poses.shape[0]}"
            )
        else:
            self.get_logger().warn(
                f"Couldn't locate cylmarker with this arm and camera configuration. Poses collected {self.gathered_robot_poses.shape[0]}"
            )

    def reset_arm(self):
        """Send arm to default position"""
        pos = Point(x=0.0, y=0.0, z=-0.12)
        ori = Quaternion(
            x=0.393899553586202,
            y=0.9179819355728568,
            z=-0.046392890942680814,
            w=-0.00000000855,
        )
        self.move_tcp_to(Pose(position=pos, orientation=ori))


    def move_tcp_to(self, target: npt.NDArray):
        """Move the TCP to the desired pose on linear trajectory.

        Args:
            target (NDArray): Desired end pose as a position-first 7-long 1D array
        """
        while self.measured_cp is None:
            rclpy.spin_once(self)

        start = pose_to_arr(self.measured_cp.pose)

        start_pos, start_ori = start[:3], start[3:]
        target_pos, target_ori = target[:3], target[3:]

        pos_dist = euclidean(start_pos, target_pos)
        pub_count = math.floor(pos_dist * self.frequency / self.velocity)

        rot_slerp = Slerp([0, pub_count], np.vstack((start_ori, target_ori)))
        states = np.arange(pub_count)

        interp_pos = np.linspace(start_pos, target_pos, pub_count)
        interp_rot = rot_slerp(states).as_quat()

        self.get_logger().debug(
            f"Moving to pose t={target_pos} R={target_ori} in {pub_count} steps."
        )

        try:
            while i := 0 < pub_count and rclpy.ok():
                next_pose = arr_to_pose(np.concatenate((interp_pos[i], interp_rot[i])))
                next_header = Header(stamp=self.get_clock().now().to_msg())
                self.servo_cp_pub.publish(PoseStamped(header=next_header, pose=next_pose))

                try:
                    self.rate.sleep()
                    i += 1
                except ROSInterruptException:
                    self.get_logger().warn("Sleep interrupted, shutting down.")
                    break
        except KeyboardInterrupt:
            self.get_logger().warn("Interruption by keyboard, shutting down.")

    def save_robot_poses(self, save_cylmarker_poses: bool = False):
        """Save robot poses to config file for auto registration and/or visualization.

        Args:
            save_cylmarker_poses (bool, optional): Whether to save the TCP poses estimated using the marker, which can be useful for visualization purposes. Defaults to False.
        """

        np.savetxt(
            os.path.join(self.poses_dir_path, f"{self.registration_id}_robot_poses.txt"),
            self.gathered_robot_poses,
        )

        if not save_cylmarker_poses:
            return

        np.savetxt(
            os.path.join(self.poses_dir_path, f"{self.registration_id}_cylmarker_poses.txt"),
            self.gathered_cylmarker_poses,
        )

    def load_robot_poses(self):
        """Load robot poses from file for auto registration."""
        if not isfile(self.auto_reg_file_path):
            msg = "Auto registration file not found"
            self.get_logger().error(msg)
            raise FileNotFoundError(msg)

        auto_reg_poses_arr = np.loadtxt(self.auto_reg_file_path)
        if auto_reg_poses_arr.ndim != 2 or auto_reg_poses_arr.shape[1] != 7:
            msg = "Invalid auto registration file, expected a 2D array with 7-long rows"
            self.get_logger().error(msg)
            raise ValueError(msg)

        for pose_arr in auto_reg_poses_arr:
            self.auto_registration_poses = np.vstack((self.auto_registration_poses, pose_arr))

    def auto_gather_poses(self):
        """Register arm with a predefined set of positions autonomously."""

        self.get_logger().info("Starting auto registration. The robot will do large movements.")

        for t in self.auto_registration_poses:
            self.move_tcp_to(t)
            time.sleep(0.5)  # Let camera settle to avoid smear frames
            self.gather_actual_position()

    def manual_gather_poses(self):
        """Wait for data collection. New poses can be collected by manually configuring
        the robot arms by pressing the clutch and releasing it while it's in the camera's FoV.
        See `cb_manip_clutch`.
        """

        self.get_logger().info("Collecting poses...")
        while rclpy.ok() and self.gathered_robot_poses.shape[0] < 15:
            rclpy.spin_once(self)

        self.get_logger().info("Poses successfully collected!")

    def register_gathered_poses(self) -> Tuple[npt.NDArray, npt.NDArray]:
        """Once enough poses are collected, find the fitting transform and plot
        the reprojection.

        Returns:
            The 3x3 rotation matrix and 1x3 translation vector
        """
        self.get_logger().debug(f"Registration started using {self.gathered_robot_poses.shape[0]}")

        cylmarker_positions = self.gathered_cylmarker_poses[:, :3]
        robot_positions = self.gathered_robot_poses[:, :3]

        R, t = rigid_transform_3D(cylmarker_positions.T, robot_positions.T)

        self.get_logger().info("Poses successfully transformed into rigid tf")

        points_transformed = np.zeros(robot_positions.shape)
        for i in range(robot_positions.shape[0]):
            p = np.dot(R, robot_positions[i, :].T) + t.T
            points_transformed[i, :] = p

        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        ax.scatter(
            cylmarker_positions[0, :],
            cylmarker_positions[1, :],
            cylmarker_positions[2, :],
            marker="o",
        )
        ax.scatter(
            points_transformed[0, :],
            points_transformed[1, :],
            points_transformed[2, :],
            marker="^",
        )

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        fig.canvas.draw()
        fig.canvas.flush_events()

        return R, t

    def save_registration(self, R: npt.NDArray, t: npt.NDArray):
        """Save registration to config file as a homogeneous transformation matrix

        Args:
            R (NDArray): Rotation matrix (3x3)
            t (NDArray): Translation vector (1x3)
        """
        self.get_logger().info("Saving robot poses...")

        # Homogenization
        t = np.vstack((t, [1]))
        R = np.vstack((R, [0, 0, 0]))
        tf = np.hstack((R, t))

        save_path = os.path.join(self.registration_dir_path, f"{self.registration_id}_reg.txt")
        np.savetxt(save_path, tf)
        self.get_logger().info(f"Collection complete, data saved at {save_path}")


def main():
    """Registrator entry point."""
    rclpy.init()
    reg = HandEyeRegistrator()

    try:
        if reg.mode == "auto":
            reg.load_robot_poses()
            reg.auto_gather_poses()
        elif reg.mode == "save" or reg.mode == "manual":
            reg.manual_gather_poses()

        if reg.mode == "auto" or reg.mode == "manual":
            R, t = reg.register_gathered_poses()
            reg.save_registration(R, t)
        elif reg.mode == "save":
            reg.save_robot_poses()

    except Exception as e:
        reg.get_logger().error(f"Unexpected error: {e}")
    finally:
        reg.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

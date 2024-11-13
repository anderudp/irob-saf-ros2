import math
import numpy as np
import rclpy
import threading
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String


class HandEyeRegistrator(Node):
    def __init__(self):
        super().__init__('hand_eye_registrator')

        self.arm = self.get_parameter('~arm')
        self.camera_registration_filename = self.get_parameter('~camera_registration_file')
        self.mode = self.get_parameter('~mode')    # simple, auto, save
        self.poses_filename = self.get_parameter('~poses_filename')

        self.poses_to_save = []
        self.robot_positions = np.zeros((0,3))
        self.cylmarker_positions = np.zeros((0,3))
        self.cylmarker_tf = None
        self.clutch_N = 0

        self.cylmarker_tf_sub = self.create_subscription(
            PoseStamped,
            'cylmarker_tf',
            self.cb_cylmarker_tf,
            10)
        self.measured_cp_sub = self.create_subscription(
            PoseStamped,
            f"/{self.arm}/measured_cp",
            self.cb_measured_cp,
            10)
        self.jaw_measured_js_sub = self.create_subscription(
            JointState,
            f"/{self.arm}/jaw/measured_js",
            self.cb_jaw_measured_js,
            10)
        self.manip_clutch_sub = self.create_subscription(
            Joy,
            f"/{self.arm}/manip_clutch",
            self.cb_manip_clutch,
            10)
    

    def cb_cylmarker_tf(self, msg):
        """Callback function for cylmarker pose."""
        self.cylmarker_tf = msg

    def cb_measured_cp(self, msg):
        """Callback function for measured_cp."""
        self.measured_cp = msg
    
    def cb_jaw_measured_js(self, msg):
        """Callback function jaw/measured_js"""
        self.measured_jaw = msg

    def cb_manip_clutch(self, msg):
        """Callback function when clutch button is pressed.
        Collect one sample from the positions.
        """
        if self.cylmarker_tf is not None and self.clutch_N > 0 and msg.buttons[0] == 0:
            self.gather_actual_position()

        self.clutch_N = self.clutch_N + 1

    
    def gather_actual_position(self):
        """Gather a single position from the camera and the robot."""
        self.get_clock().sleep_for(Duration(seconds=0.5))  # This might not work
        if ((self.get_clock().now().to_sec() - self.cylmarker_tf.header.stamp.to_sec()) < 0.4):  # This might not work either
            robot_pos = np.array([self.measured_cp.pose.position.x,
                                self.measured_cp.pose.position.y,
                                self.measured_cp.pose.position.z]).T
            cylmarker_pos = np.array([self.cylmarker_tf.pose.position.x,
                                    self.cylmarker_tf.pose.position.y,
                                    self.cylmarker_tf.pose.position.z]).T
            
            if self.mode == "save":
                self.poses_to_save.append(self.measured_cp)

            self.robot_positions = np.vstack((self.robot_positions, robot_pos))
            self.cylmarker_positions = np.vstack((self.cylmarker_positions, cylmarker_pos))

            print("Positions collected: " + str(self.robot_positions.shape[0]))
        else:
            print("Couldn't locate fiducials in this setup.")


if __name__ == '__main__':
    rclpy.init()
    
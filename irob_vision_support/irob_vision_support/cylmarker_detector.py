# Cylindrical marker hand-eye calibration
# Based on "An Enhanced Marker Pattern that Achieves 
# Improved Accuracy in Surgical Tool Tracking"
# (Cartucho et al., 2021)

import numpy as np
import rclpy
import cv2
import os
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from cylmarker_utils.load_data import load_config_and_cam_calib_data, load_pttrn_and_marker_data
from cylmarker_utils.pose_estimation import pose_estimation
from tf_transformations import quaternion_from_matrix
from rclpy.time import Time


class CylmarkerDetector(Node):
    def __init__(self):
        super().__init__('cylmarker_detector')

        self.cylmarker_tf_pub = self.create_publisher(
            PoseStamped,
            "cylmarker_tf",
            10)
        
        self.declare_parameter('config_folder_path', "")
        self.declare_parameter('data_path', "")
        
        self.config_folder_path = self.get_parameter('config_folder_path').get_parameter_value().string_value
        self.cam_calib_config_path = self.config_folder_path + "/camera_info/bark-logitech-c270.yaml"
        self.cylmarker_config_path = self.config_folder_path + "/cylmarker/config.yaml"
        self.marker_config_path = self.config_folder_path + "/cylmarker/marker.yaml"
        self.pattern_config_path = self.config_folder_path + "/cylmarker/pattern.yaml"

        self.data_path = self.get_parameter('data_path').get_parameter_value().string_value
        self.raw_images_path = self.data_path + "/raw_images/"
        self.processed_images_path = self.data_path + "/processed_images/"

        # Ensure image folders exist
        if not os.path.exists(self.raw_images_path):
            os.makedirs(self.raw_images_path)
        if not os.path.exists(self.processed_images_path):
            os.makedirs(self.processed_images_path)
        
        self.image_stamp: Time = self.get_clock().now()
        

    def estimate(self, image: np.ndarray):
        """Attempts pose estimation on the raw image,
        then publishes the result.
        """

        data_config, data_cam_calib = load_config_and_cam_calib_data(
            config_file_path=self.cylmarker_config_path, 
            cam_calib_file_path=self.cam_calib_config_path)
        
        data_pattern, data_marker = load_pttrn_and_marker_data(
            pttrn_file_path=self.pattern_config_path,
            marker_file_path=self.marker_config_path)

        pose_pred = pose_estimation.estimate_poses(
            image, 
            data_cam_calib, 
            data_config, 
            data_pattern, 
            data_marker, 
            debug_ims_path=self.processed_images_path)  # Set to None if debug images are not needed

        #self.get_logger().log(str(pose_pred), 20)

        p = PoseStamped()
        q = quaternion_from_matrix(pose_pred)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        p.pose.position.x = pose_pred[0][3]
        p.pose.position.y = pose_pred[1][3]
        p.pose.position.z = pose_pred[2][3]
        p.header.stamp = self.image_stamp.to_msg()
        self.cylmarker_tf_pub.publish(p)


    def take_photo_usb_webcam(self, save_raw: bool = False):
        capture = cv2.VideoCapture(0)
        success = False
        frame = None
        while not success:
            success, frame = capture.read()
        capture.release()

        self.image_stamp = self.get_clock().now()
        if save_raw:
            cv2.imwrite(f"{self.raw_images_path}/{self.image_stamp.nanoseconds}_raw.png", frame)

        return frame
    

def main():
    rclpy.init()
    detector = CylmarkerDetector()
    #detector.get_logger().log(detector.get_parameter('data_path').get_parameter_value().string_value, 20)
    image = detector.take_photo_usb_webcam(save_raw=True)
    #image = cv2.imread("/root/ros2_ws/src/irob-saf-ros2/irob_vision_support/data/raw_images/2024-12-06-160510.jpg")
    detector.estimate(image)

    try:
        rclpy.spin(detector)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
# Cylindrical marker hand-eye calibration
# Based on "An Enhanced Marker Pattern that Achieves 
# Improved Accuracy in Surgical Tool Tracking"
# (Cartucho et al., 2021)

import numpy as np
import rclpy
import cv2
import os
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
import pyrealsense2.pyrealsense2 as rs
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
        
        self.config_folder_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', "")
        self.cam_calib_config_path = self.config_folder_path + "camera_info/pappad-jendoscope-aliexpress.yaml"
        self.cylmarker_config_path = self.config_folder_path + "cylmarker/config.yaml"
        self.marker_config_path = self.config_folder_path + "cylmarker/marker.yaml"
        self.pattern_config_path = self.config_folder_path + "cylmarker/pattern.yaml"

        self.data_path = os.path.join(os.path.dirname(__file__), '..', '..', 'data', "")
        self.raw_images_path = self.data_path + "raw_images/"
        self.processed_images_path = self.data_path + "processed_images/"
        
        self.image_stamp: Time = None
        


    def estimate(self, image: np.ndarray):
        """Attempts pose estimation on the raw image,
        then publishes the result.
        """

        # Estimate poses
        data_config, data_cam_calib = load_config_and_cam_calib_data(
            config_file_path=self.cylmarker_config_path, 
            cam_calib_file_path=self.cam_calib_config_path)

        data_pattern, data_marker = load_pttrn_and_marker_data(
            pattern_file_path=self.pattern_config_path,
            marker_file_data=self.marker_config_path)

        # Realsense specific, use data_cam_calib for other cameras
        dist_coeff_data = self.intrinsics.coeffs

        cam_matrix = [[self.intrinsics.fx, 0, self.intrinsics.ppx],
                      [0, self.intrinsics.fy, self.intrinsics.ppy],
                      [0, 0, 1]]
        
        pose_pred = pose_estimation.estimate_poses(image, cam_matrix, dist_coeff_data, data_config, data_pattern, data_marker)

        p = PoseStamped()
        q = quaternion_from_matrix(pose_pred)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        p.pose.position.x = pose_pred[0][3]
        p.pose.position.y = pose_pred[1][3]
        p.pose.position.z = pose_pred[2][3]
        p.header.stamp = self.latest_stamp
        self.cylmarker_tf_pub.publish(p)

    def take_photo(self, save_raw: bool = False) -> np.ndarray:
        # Attempt to take a photo
        capture = cv2.VideoCapture(0)
        success = False
        frame = None
        while not success:
            success, frame = capture.read()
        capture.release()

        self.image_stamp = self.get_clock().now()
        if save_raw:
            cv2.imwrite(f"{self.raw_images_path}/{self.image_stamp.nanoseconds}_raw.png", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        return frame

def main():
    rclpy.init()
    detector = CylmarkerDetector()
    #image = detector.take_photo_realsense()
    image = detector.take_photo(save_raw=True)
    detector.estimate(image)

    try:
        rclpy.spin(detector)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
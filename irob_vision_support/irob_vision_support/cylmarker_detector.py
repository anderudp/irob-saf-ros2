"""Cylindrical marker hand-eye calibration
Based on "An Enhanced Marker Pattern that Achieves Improved Accuracy in Surgical Tool Tracking"
(Cartucho et al., 2021)"""

import os
import numpy as np
import numpy.typing as npt
import rclpy
import rclpy.logging
import cv2
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from cylmarker_utils.load_data import load_config_and_cam_calib_data, load_pttrn_and_marker_data
from cylmarker_utils.pose_estimation import pose_estimation
from tf_transformations import quaternion_from_matrix
from irob_utils.conversions import arr_to_pose


class CylmarkerDetector(Node):
    """Detects the cylindrical marker and publishes its pose in camera space"""
    def __init__(self):
        super().__init__('cylmarker_detector')

        self.registration_id = self.get_parameter("registration_id").get_parameter_value().string_value
        self.detector_images_dir_path = self.get_parameter('detector_images_dir_path').get_parameter_value().string_value
        self.cam_calib_file_path = self.get_parameter('cam_calib_file_path').get_parameter_value().string_value
        self.detector_config_file_path = self.get_parameter('detector_config_file_path').get_parameter_value().string_value
        self.marker_config_file_path = self.get_parameter('marker_config_file_path').get_parameter_value().string_value
        self.pattern_config_file_path = self.get_parameter('pattern_config_file_path').get_parameter_value().string_value

        self.im_path_stem = os.path.join(self.detector_images_dir_path, self.registration_id)
        self.cylmarker_tf_pub = self.create_publisher(
            PoseStamped,
            "cylmarker_tf",
            10
        )


    def estimate(self, image: npt.NDArray):
        """Attempts pose estimation on the raw image,
        then publishes the result.
        """

        data_config, data_cam_calib = load_config_and_cam_calib_data(
            config_file_path=self.detector_config_file_path,
            cam_calib_file_path=self.cam_calib_file_path
        )

        data_pattern, data_marker = load_pttrn_and_marker_data(
            pttrn_file_path=self.pattern_config_file_path,
            marker_file_path=self.marker_config_file_path
        )

        pose_pred = pose_estimation.estimate_poses(
            image,
            data_cam_calib,
            data_config,
            data_pattern,
            data_marker,
            debug_im_path_stem=self.im_path_stem # Set to None if debug images are not needed
        )

        if pose_pred is None:
            h = Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="invalid"
            )
            p = PoseStamped(header=h)
            self.cylmarker_tf_pub.publish(p)
        else:
            h = Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=self.registration_id
            )
            pos = pose_pred[:3, 3]
            rot = np.array(quaternion_from_matrix(pose_pred))
            p = PoseStamped(
                header=h,
                pose=arr_to_pose(np.concatenate((pos, rot)))
            )
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
            cv2.imwrite(os.path.join(self.im_path_stem, "raw.jpg"), frame)

        return frame
    

def main():
    rclpy.init()
    detector = CylmarkerDetector()
    #detector.get_logger().log(detector.get_parameter('data_path').get_parameter_value().string_value, 20)
    #image = cv2.imread("/root/ros2_ws/src/irob-saf-ros2/irob_vision_support/data/raw_images/2024-12-06-160510.jpg")
    try:
        while rclpy.ok():
            image = detector.take_photo_usb_webcam(save_raw=True)
            #image = cv2.imread("/root/ros2_ws/src/irob-saf-ros2/irob_vision_support/data/raw_images/2024-12-06-160510.jpg")
            detector.estimate(image)
            cv2.imshow("cylmarker", image)
            cv2.waitKey(1)
            rclpy.spin_once(detector)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

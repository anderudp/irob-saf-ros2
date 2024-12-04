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

class CylmarkerDetector(Node):
    def __init__(self):
        super().__init__('cylmarker_detector')

        self.cylmarker_tf_pub = self.create_publisher(
            PoseStamped,
            "cylmarker_tf",
            10)
        
        self.config_folder_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', "")
        self.cam_calib_config_path = self.config_folder_path + "camera_info/camera_dvrk_left.yaml"  # TODO: Get realsense calib
        self.cylmarker_config_path = self.config_folder_path + "cylmarker/config.yaml"
        self.marker_config_path = self.config_folder_path + "cylmarker/marker.yaml"
        self.pattern_config_path = self.config_folder_path + "cylmarker/pattern.yaml"

        self.data_path = os.path.join(os.path.dirname(__file__), '..', '..', 'data', "")
        self.raw_images_path = self.data_path + "raw_images/"
        self.processed_images_path = self.data_path + "processed_images/"
        
        # Camera init
        self.width = 640
        self.height = 480
        self.fps = 30 #30
        self.clipping_distance_in_meters = 0.30
        self.exposure = 600.0 #300.0
        self.tr_seq = 0
        self.z_offset = 0.004 #m
        self.points_in_cam_base = np.array([[0.036,0.0,0.0],
                                            [-0.036,0.0,0.0],
                                            [0.0,0.036,0.0],
                                            [0.0,-0.036,0.0]]).T
        
        self.context = rs.context()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.intrinsics = None

        self.latest_stamp = None


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


    def take_photo_realsense(self, save_raw: bool = False, save_processed: bool = True) -> np.ndarray:
        """Connect to a RealSense camera and takes a photo,
        with distant pixels discarded. Returns the processed photo.
        """

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        self.set_exposure(self.exposure)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        clipping_distance = self.clipping_distance_in_meters / depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)
        bg_removed = None

        # Take picture
        try:
            while rclpy.ok() and bg_removed is None:
                # Get frameset of color and depth
                frames = self.pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame()
                # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Remove background - Set pixels further than clipping_distance to grey
                grey_color = 0
                depth_image_3d = np.dstack((depth_image,depth_image,depth_image))
                #depth image is 1 channel, color is 3 channels
                bg_removed = np.where((depth_image_3d > clipping_distance) 
                                      | (depth_image_3d <= 0), grey_color, color_image)


                self.intrinsics = (aligned_depth_frame.profile.as_video_stream_profile().get_intrinsics())
                self.latest_stamp = self.get_clock().now().nanoseconds
                if save_raw:
                    cv2.imwrite(f"{self.raw_images_path}/{self.latest_stamp}_raw.png", cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
                if save_processed:
                    cv2.imwrite(f"{self.raw_images_path}/{self.latest_stamp}_bgremoved.png", cv2.cvtColor(bg_removed, cv2.COLOR_BGR2RGB))
                
        finally:
            self.pipeline.stop()

        return bg_removed


def main():
    rclpy.init()
    detector = CylmarkerDetector()
    image = detector.take_photo_realsense()
    detector.estimate(image)

    try:
        rclpy.spin(detector)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
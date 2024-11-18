import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
import pyrealsense2 as rs


class FiducialDetector(Node):
    def __init__(self):
        super().__init__('fiducial_detector')

        self.cylmarker_tf_pub = self.create_publisher(
            PoseStamped,
            "cylmarker_tf",
            10)

        self.width = 640
        self.height = 480
        self.fps = 30 #30
        self.clipping_distance_in_meters = 0.30
        self.exposure = 600.0 #300.0
        self.tr_seq = 0
        self.z_offset = 0.004   # m
        self.points_in_cam_base = np.array([[0.036,0.0,0.0],
                                            [0-.036,0.0,0.0],
                                            [0.0,0.036,0.0],
                                            [0.0,-0.036,0.0]]).T
        
        self.context = rs.context()
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, self.width,
                                    self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width,
                                    self.height, rs.format.bgr8, self.fps)


if __name__ == '__main__':
    rclpy.init()
    
    try:
        rclpy.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()
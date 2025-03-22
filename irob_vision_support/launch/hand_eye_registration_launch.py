"""Hand-eye registration launch file"""

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Provides launch description for hand-eye registration

    Returns:
        LaunchDescription: Arguments, nodes for hand-eye registration
    """

    vision_pkg_path = get_package_share_directory("irob_vision_support")

    registration_id = DeclareLaunchArgument(
        "registration_id",
        default_value=datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
        description="ID of the current registration, used to uniquely identify the data produced thereby"
    )

    arm = DeclareLaunchArgument(
        "arm",
        default_value="PSM1",
        description="The arm whose pose is being estimated"
    )

    mode = DeclareLaunchArgument(
        "mode",
        default_value="manual",
        description="Registration method. Use `manual` to manually position the arm, `auto` to iterate over a previously configured set of poses, and `save` to configure arm poses for auto-registration",
        choices=["manual", "auto", "save"],
    )

    save_cylmarker_poses = DeclareLaunchArgument(
        "save_cylmarker_poses",
        default_value=False,
        description="Whether to save the cylmarker poses along with the robot poses in `save` mode.",
    )

    frequency = DeclareLaunchArgument(
        "frequency",
        default_value=10,
        description="Frequency for pose publication and estimation",
    )

    velocity = DeclareLaunchArgument(
        "velocity",
        default_value=0.05,
        description="TCP linear velocity for auto-calibration",
    )

    detector_images_dir_path = DeclareLaunchArgument(
        "detector_images_dir_path",
        default_value=os.path.join(vision_pkg_path, "data", "detector_images"),
        description="Directory where the images of the intermediary phases of detection are to be saved",
    )

    registration_dir_path = DeclareLaunchArgument(
        "registration_dir_path",
        default_value=os.path.join(vision_pkg_path, "data", "registration"),
        description="Directory where the generated registration file will be saved",
    )

    poses_dir_path = DeclareLaunchArgument(
        "poses_dir_path",
        default_value=os.path.join(vision_pkg_path, "config", "poses"),
        description="Directory where the actual robot poses and detected cylmarker poses will be saved",
    )

    auto_reg_file_path = DeclareLaunchArgument(
        "auto_reg_file_path",
        default_value=os.path.join(vision_pkg_path, "config", "poses", "0_robot_poses.txt"),
        description="File containing the poses used for auto calibration",
    )

    cam_calib_file_path = DeclareLaunchArgument(
        "cam_calib_file_path",
        default_value=os.path.join(vision_pkg_path, "config", "camera_info", "pappad-jendoscope-aliexpress.yaml"),
        description="File containing the calibration data for the camera used for registration",
    )

    detector_config_file_path = DeclareLaunchArgument(
        "detector_config_file_path",
        default_value=os.path.join(vision_pkg_path, "config", "cylmarker", "config.yaml"),
        description="File containing the parameters used by the cylmarker detector",
    )

    marker_config_file_path = DeclareLaunchArgument(
        "marker_config_file_path",
        default_value=os.path.join(vision_pkg_path, "config", "cylmarker", "marker.yaml"),
        description="File containing the pose information of the cylmarker, used for deprojection",
    )

    pattern_config_file_path = DeclareLaunchArgument(
        "pattern_config_file_path",
        default_value=os.path.join(vision_pkg_path, "config", "cylmarker", "pattern.yaml"),
        description="File containing the pattern information of the cylmarker, used to uniquely identify keypoints",
    )

    registrator = Node(
        package="irob_vision_support",
        executable="hand_eye_registrator",
        name="hand_eye_registrator",
        parameters=[{
            "registration_id": LaunchConfiguration("registration_id"),
            "arm": LaunchConfiguration("arm"),
            "mode": LaunchConfiguration("mode"),
            "save_cylmarker_poses": LaunchConfiguration("save_cylmarker_poses"),
            "frequency": LaunchConfiguration("frequency"),
            "velocity": LaunchConfiguration("velocity"),
            "registration_dir_path": LaunchConfiguration("registration_dir_path"),
            "poses_dir_path": LaunchConfiguration("poses_dir_path"),
            "auto_reg_file_path": LaunchConfiguration("auto_reg_file_path"),
        }]
    )

    detector = Node(
        package="irob_vision_support",
        executable="cylmarker_detector",
        name="cylmarker_detector",
        parameters=[{
            "registration_id": LaunchConfiguration("registration_id"),
            "detector_images_dir_path": LaunchConfiguration("detector_images_dir_path"),
            "cam_calib_file_path": LaunchConfiguration("cam_calib_file_path"),
            "detector_config_file_path": LaunchConfiguration("detector_config_file_path"),
            "marker_config_file_path": LaunchConfiguration("marker_config_file_path"),
            "pattern_config_file_path": LaunchConfiguration("pattern_config_file_path"),
        }]
    )

    return LaunchDescription(
        [
            registration_id,
            arm,
            mode,
            save_cylmarker_poses,
            frequency,
            velocity,
            detector_images_dir_path,
            registration_dir_path,
            poses_dir_path,
            auto_reg_file_path,
            cam_calib_file_path,
            detector_config_file_path,
            marker_config_file_path,
            pattern_config_file_path,
            registrator,
            detector,
        ]
    )

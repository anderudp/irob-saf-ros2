"""Utilities for converting between different types of objects"""

import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import Pose, Point, Quaternion

# TODO: Add assertion to array size if numpy>=2.1.0 support is added as per https://github.com/numpy/numpy/pull/26081
def arr_to_pose(arr_in: npt.NDArray) -> Pose:
    """Converts a Numpy array into a Pose object.

    Args:
        arr_in (NDArray): Array to convert to a Pose. Must be 1 dimensional and of length 7

    Returns:
        geometry_msgs/Pose: The converted Pose object
    """
    arr_in = arr_in.flatten()
    if arr_in.size != 7:
        msg = f"Cannot convert specified array to Pose. Expected array of length 7, got {arr_in.size}"
        raise ValueError(msg)

    new_pos = Point(x=arr_in[0], y=arr_in[1], z=arr_in[2])
    new_ori = Quaternion(x=arr_in[3], y=arr_in[4], z=arr_in[5], w=arr_in[6])
    return Pose(position=new_pos, orientation=new_ori)

def pose_to_arr(pose_in: Pose) -> npt.NDArray:
    """Converts a Pose int a Numpy array of length 7.

    Args:
        pose_in (geometry_msgs/Pose): Pose to convert to an array.

    Returns:
        A 7-long array representing the pose
    """
    return np.array(
        [
            pose_in.position.x,
            pose_in.position.y,
            pose_in.position.z,
            pose_in.orientation.x,
            pose_in.orientation.y,
            pose_in.orientation.z,
            pose_in.orientation.w,
        ]
    )

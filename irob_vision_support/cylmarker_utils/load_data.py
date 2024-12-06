import errno
import os
import glob
import math

import yaml
from natsort import natsorted

def is_path_dir(string):
    if os.path.isdir(string):
        return string
    else:
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), string)


def is_path_file(string):
    if os.path.isfile(string):
        return string
    else:
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), string)


def load_yaml_data(path):
    if is_path_file(path):
        with open(path) as f_tmp:
            return yaml.load(f_tmp, Loader=yaml.FullLoader)


def load_config_and_cam_calib_data(config_file_path: str, cam_calib_file_path: str):
    # Load config file data
    config_file_data = load_yaml_data(config_file_path)

    # Load camera intrinsic matrix and distortion coefficients
    cam_calib_data = load_yaml_data(cam_calib_file_path)

    return config_file_data, cam_calib_data


def load_pttrn_and_marker_data(pttrn_file_path: str, marker_file_path: str):
    # Load pattern data
    pttrn_file_data = load_yaml_data(pttrn_file_path)
    # Load marker data
    marker_file_data = load_yaml_data(marker_file_path)
    return pttrn_file_data, marker_file_data


def load_img_paths(img_dir_path, img_format):
    if ".." in img_dir_path:
        img_dir_path = os.path.abspath(img_dir_path)
    if is_path_dir(img_dir_path):
        img_paths = os.path.join(img_dir_path, '*{}'.format(img_format))
        img_paths_sorted = natsorted(glob.glob(img_paths))
        return img_paths_sorted
    

def get_marker_diameter(config_file_data):
    cyl_diam = config_file_data['cyl_diameter_mm']
    paper_thickness = config_file_data['printing_paper_thickness_mm']
    return cyl_diam + 2.0 * paper_thickness


def get_marker_mm_per_pixel(marker_diameter, marker_width):
    marker_perimeter = marker_diameter * math.pi
    return marker_perimeter/marker_width


def get_marker_radius_and_mmperpixel(config_file_data, marker_width):
    diam = get_marker_diameter(config_file_data)
    radius = diam/2.
    mm_per_pixel = get_marker_mm_per_pixel(diam, marker_width)
    return radius, mm_per_pixel




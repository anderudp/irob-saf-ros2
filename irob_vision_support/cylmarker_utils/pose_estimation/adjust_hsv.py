# HSV adjustment utility. Run this as a regular Python script.

config_file_path = "/root/ros2_ws/src/irob-saf-ros2/irob_vision_support/config/cylmarker/config.yaml"
images_path = "/root/ros2_ws/src/irob-saf-ros2/irob_vision_support/data/raw_images"
img_format = ".jpg"

import sys
sys.path.append('/root/ros2_ws/src/irob-saf-ros2/irob_vision_support')
from cylmarker_utils import load_data
from cylmarker_utils.pose_estimation import img_segmentation

import cv2 as cv
import numpy as np

title_window = 'Adjust HSV range!'
img_paths = []
h_min = -1
h_max = -1
s_min = -1
v_min = -1
im_ind = 0
mouse_x = 0
mouse_y = 0


def check_image(im, im_path):
    if im is None:
        print('Error opening the image {}'.format(im_path))
        exit()


def trackbar_callback_im(im_ind_new):
    global im_ind
    im_ind = im_ind_new
    im_path = img_paths[im_ind]
    im = cv.imread(im_path, cv.IMREAD_COLOR)
    check_image(im, im_path) # check if image was sucessfully read
    im_hsv = cv.cvtColor(im, cv.COLOR_BGR2HSV)
    # Do segmentation with current values
    mask_marker_bg, _ = img_segmentation.get_marker_background_hsv(im_hsv, h_min, h_max, s_min, v_min)
    if mask_marker_bg is not None:
        im_copy = im.copy()
        # Highlight green part in red
        alpha = 0.7
        mask_bg_red = np.zeros_like(im_copy)
        mask_bg_red[:,:,2] = mask_marker_bg
        im_copy = cv.addWeighted(im_copy, 1.0, mask_bg_red, alpha, 0)
        # Draw current HSV value
        pt_hsv = im_hsv[mouse_y, mouse_x].tolist()
        text = "HSV = [{}, {}, {}]".format(pt_hsv[0], pt_hsv[1], pt_hsv[2])
        cv.putText(im_copy, text, (mouse_x, mouse_y), 0, 1, (0, 0, 0), 5) # Black text border
        cv.putText(im_copy, text, (mouse_x, mouse_y), 0, 1, im[mouse_y, mouse_x].tolist(), 2)
        cv.imshow(title_window, im_copy)


def trackbar_callback_h_min(h_min_new):
    global h_min
    h_min = h_min_new
    trackbar_callback_im(im_ind)


def trackbar_callback_h_max(h_max_new):
    global h_max
    h_max = h_max_new
    trackbar_callback_im(im_ind)


def trackbar_callback_s_min(s_min_new):
    global s_min
    s_min = s_min_new
    trackbar_callback_im(im_ind)


def trackbar_callback_v_min(v_min_new):
    global v_min
    v_min = v_min_new
    trackbar_callback_im(im_ind)


def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y
    mouse_x = x
    mouse_y = y
    trackbar_callback_im(im_ind)


def improve_segmentation():
    global h_min, h_max, s_min, v_min
    global img_paths

    # Initialize values
    try:
        config_file_data = load_data.load_yaml_data(config_file_path)
    except(FileNotFoundError):
        raise FileNotFoundError(f"Could not find config file at path {config_file_path}!")
    
    try:
        img_paths = load_data.load_img_paths(images_path, img_format)
    except(FileNotFoundError):
        raise FileNotFoundError(f"Could not find images at path {images_path}!")

    if len(img_paths) == 0:
        raise FileNotFoundError(f"The image path exists, but no images were found! Are you sure the images are {img_format} files?")
    
    h_min = config_file_data['h_min']
    h_max = config_file_data['h_max']
    s_min = config_file_data['s_min']
    v_min = config_file_data['v_min']

    print('All the images will share the same HSV range.')
    print('Press any [key] when finished.')

    cv.namedWindow(title_window, cv.WINDOW_NORMAL)
    cv.createTrackbar("Image", title_window, 0, len(img_paths) - 1, trackbar_callback_im)
    cv.createTrackbar("h_min", title_window, h_min, 180, trackbar_callback_h_min)
    cv.createTrackbar("h_max", title_window, h_max, 180, trackbar_callback_h_max)
    cv.createTrackbar("s_min", title_window, s_min, 255, trackbar_callback_s_min)
    cv.createTrackbar("v_min", title_window, v_min, 255, trackbar_callback_v_min)
    if len(img_paths) > 0:
        trackbar_callback_im(0)
        cv.setMouseCallback(title_window, mouse_callback)
        cv.waitKey()
        print('\nDone! Please modify these values in the `config.yaml` file:')
        print('h_min: {}'.format(h_min))
        print('h_max: {}'.format(h_max))
        print('s_min: {}'.format(s_min))
        print('v_min: {}'.format(v_min))
    else:
        print('ERROR: No images found')


def main():
    improve_segmentation()


if __name__ == "__main__":
    main()
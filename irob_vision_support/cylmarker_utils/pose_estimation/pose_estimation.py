from cylmarker_utils import load_data, keypoints
from cylmarker_utils.pose_estimation import img_segmentation
from cylmarker_utils.keypoints import Pattern

import cv2 as cv
import numpy as np
import os
import time

def show_sgmntd_bg_and_fg(im, mask_marker_bg, mask_marker_fg):
    im_copy = im.copy()
    alpha = 0.4
    # First we show the background part only
    mask_marker_bg = cv.subtract(mask_marker_bg, mask_marker_fg);
    mask_bg_blue = np.zeros_like(im_copy)
    mask_bg_blue[:,:,0] = mask_marker_bg
    im_copy = cv.addWeighted(im_copy, 1.0, mask_bg_blue, alpha, 0)
    # Then we show the foreground part
    alpha = 0.7
    mask_fg_red = np.zeros_like(im_copy)
    mask_fg_red[:,:,2] = mask_marker_fg
    im_copy = cv.addWeighted(im_copy, 1.0, mask_fg_red, alpha, 0)
    cv.imshow("Segmentation | Blue: background | Red: foreground", im_copy)
    cv.waitKey(0)


def show_contours_and_lines_and_centroids(im, pttrn):
    blue = (255, 0, 0)
    yellow=(124,225,255)
    red = (0, 0, 255)
    for sqnc in pttrn.list_sqnc:
        if sqnc.sqnc_id != -1:
            """ draw contours """
            for kpt in sqnc.list_kpts:
                cntr = kpt.cntr
                cv.drawContours(im, [cntr], -1, blue, 1)
            """ draw line between first and last kpt """
            u_0, v_0 = sqnc.list_kpts[0].get_centre_uv()
            u_1, v_1 = sqnc.list_kpts[-1].get_centre_uv()
            im = cv.line(im, (int(u_0), int(v_0)), (int(u_1), int(v_1)), yellow, 2, cv.LINE_AA) # lines
            """ draw centroids """
            for kpt in sqnc.list_kpts:
                u, v = kpt.get_centre_uv()
                im = cv.circle(im, (int(round(u)), int(round(v))), radius=2, color=red, thickness=-1)
    return im
    #cv.waitKey(0)


def show_axis(im, transf, rvecs, tvecs, cam_matrix, dist_coeff, length_m):
    axis = np.float32([[0, 0, 0], [length_m,0,0], [0,length_m,0], [0,0,length_m]]).reshape(-1,3)
    # Understand which axis to draw first
    dist_z = []
    for target_T_pt in axis[1:]:
        target_T_pt = np.vstack((target_T_pt.reshape(3,1), [1]))
        cam_T_pt = np.matmul(transf, target_T_pt)
        dist_z.append(-cam_T_pt[2,0]) # - so that we sort from further to closer
    args = np.argsort(dist_z)
    imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, cam_matrix, dist_coeff)
    frame_centre = tuple(np.rint(imgpts[0]).astype(int).ravel())
    thickness = 4
    colors = [(0,0,255), (0,255,0), (255,0,0)] # Red, Green, Blue
    for i in args:
        cv.line(im, frame_centre, tuple(np.rint(imgpts[i+1]).astype(int).ravel()), colors[i], thickness, cv.LINE_AA)
    return im


def get_transf_inv(transf):
    # ref: https://math.stackexchange.com/questions/152462/inverse-of-transformation-matrix
    r = transf[0:3, 0:3]
    t = transf[0:3, 3]
    r_inv = np.transpose(r) # Orthogonal matrix so the inverse is its transpose
    t_new = np.matmul(-r_inv, t).reshape(3, 1)
    transf_inv = np.concatenate((r_inv, t_new), axis = 1)
    return transf_inv


def save_pts_info(im_path, pnts_3d_object, pnts_2d_image):
    filename = '{}.pts3d.txt'.format(im_path)
    np.savetxt(filename, (np.squeeze(pnts_3d_object)), fmt="%s", delimiter=',')
    filename = '{}.pts2d.txt'.format(im_path)
    np.savetxt(filename, (np.squeeze(pnts_2d_image)), fmt="%s", delimiter=',')


def save_pose(img_format, im_path, mat):
    filename = im_path.replace(img_format, '.txt')
    np.savetxt(filename, (mat), fmt="%s", delimiter=',')


def show_reproj_error_image(im, pts2d_filtered, pts2d_projected):
    red = (0, 0, 255)
    green = (0, 255, 0)
    for pt2d_d, pt2d_p in zip(pts2d_filtered[:,0], pts2d_projected[:,0]):
        pt2d_d = (int(round(pt2d_d[0])), int(round(pt2d_d[1])))
        pt2d_p = (int(round(pt2d_p[0])), int(round(pt2d_p[1])))
        im = cv.line(im, pt2d_d, pt2d_p, color=red, thickness=1, lineType=cv.LINE_AA)
        im = cv.circle(im, pt2d_d, radius=1, color=red, thickness=-1)
        im = cv.circle(im, pt2d_p, radius=1, color=green, thickness=-1)
    cv.imshow("image", im)
    cv.waitKey(0)


def get_reprojection_error(pts3d, rvec, tvec, inliers, cam_matrix, dist_coeff, pnts2d, show_reproj_error, im):
    """ This function calculates the reprojection error of the inlier points """
    # Filter the inlier points
    n_inliers, _ = inliers.shape
    pts3d_filtered = np.zeros((n_inliers, pts3d.shape[1], pts3d.shape[2]), dtype=np.float)
    pts2d_filtered = np.zeros((n_inliers, pnts2d.shape[1], pnts2d.shape[2]), dtype=np.float)
    for ind_new, ind_old in enumerate(inliers[:,0]):
        pts3d_filtered[ind_new] = pts3d[ind_old]
        pts2d_filtered[ind_new] = pnts2d[ind_old]
    # Project 3D points into the 2D image
    pts2d_projected, jacobian = cv.projectPoints(pts3d_filtered, rvec, tvec, cam_matrix, dist_coeff)
    # Compare projected 2D points with the detected 2D points
    ## First ensure that they have the same shape
    pnts2d_detected = np.reshape(pts2d_filtered, pts2d_projected.shape)
    if show_reproj_error:
        show_reproj_error_image(im, pnts2d_detected, pts2d_projected)
    se = (pts2d_projected - pnts2d_detected) ** 2
    sse = np.sum(se)
    reproj_error = np.sqrt(sse / n_inliers) # Using the same formula as in OpenCV's calibration documentation
    return reproj_error # in [pixels]


def draw_detected_and_projected_features(rvecs, tvecs, cam_matrix, dist_coeff, pttrn, im):
    for sqnc in pttrn.list_sqnc:
        if sqnc.sqnc_id != -1:
            """
                We will draw the detected and projected contours of each feature in a sequence.
            """
            for kpt in sqnc.list_kpts:
                # First, we draw the detected contour (in green)
                cntr = kpt.cntr
                #im = cv.drawContours(im, [cntr], -1, [0, 255, 0], -1)
                im = cv.drawContours(im, [cntr], -1, [0, 255, 0], 1)
                # Then, we calculate + draw the projected contour (in red)
                corners_3d = np.float32(kpt.xyz_corners).reshape(-1,3)
                imgpts, jac = cv.projectPoints(corners_3d, rvecs, tvecs, cam_matrix, dist_coeff)
                imgpts = np.asarray(imgpts, dtype=np.int32)
                #im = cv.fillPoly(im, [imgpts], [0, 0, 255])
                im = cv.polylines(im, [imgpts], True, [0, 0, 255], thickness=1, lineType=cv.LINE_AA)
    return im


def estimate_poses(image, cam_calib_data, config_file_data, data_pttrn, data_marker, debug_im_path_stem = None):
    """Estimates the position of the marker.
    Returns its homogenous pose.
    """
    if image is None:
        raise Exception("Empty image provided.")
    
    if debug_im_path_stem is not None:
        os.makedirs(debug_im_path_stem)
        cv.imwrite(os.path.join(debug_im_path_stem, "read_image.jpg"), image)
    
    ## Load pattern data
    sqnc_max_ind = len(data_pttrn) - 1
    sequence_length = len(data_pttrn['sequence_0']['code'])

    ## Load camera matrix and distortion coefficients
    cam_matrix = cam_calib_data['intrinsic']
    cam_matrix = np.reshape(cam_matrix, (3, 3))
    dist_coeff_data = cam_calib_data['distortion']
    dist_coeff_np = np.array(dist_coeff_data)

    """ Step I - Undistort the input image """
    im = cv.undistort(image, cam_matrix, dist_coeff_np) # undistort each new image
    dist_coeff = None # we don't need to undistort again the same image

    if debug_im_path_stem is not None:
        cv.imwrite(os.path.join(debug_im_path_stem, "undistorted_img.jpg"), im)

    """ Step II - Segment the marker and detect features """
    mask_marker_bg, mask_marker_fg = img_segmentation.marker_segmentation(im, config_file_data, debug_im_path_stem)
    if mask_marker_bg is None:
        return None
    #     raise Exception("Marker could not be detected. Your HSV range configuration may be insufficient.")
    
    if debug_im_path_stem is not None:
        cv.imwrite(os.path.join(debug_im_path_stem, "mask_marker_bg.jpg"), mask_marker_bg)
        cv.imwrite(os.path.join(debug_im_path_stem, "mask_marker_fg.jpg"), mask_marker_fg)
    
    # Draw segmented background and foreground
    #show_sgmntd_bg_and_fg(im, mask_marker_bg, mask_marker_fg)

    """ Step III - Identify features """
    pttrn: Pattern = keypoints.find_keypoints(im, mask_marker_fg, config_file_data, sqnc_max_ind, sequence_length, data_pttrn, data_marker)
    if pttrn is None:
        return None
    #     raise Exception("Pattern could not be sufficiently read. Your HSV range config, dot pattern threshold or sequence config may be incorrect.")

    # Estimate pose
    # Draw contours and lines (for visualization)
    if debug_im_path_stem is not None:
        lines_countours_image = show_contours_and_lines_and_centroids(im, pttrn)
        cv.imwrite(os.path.join(debug_im_path_stem, "lines_countours_image.jpg"), lines_countours_image)

    pnts_3d_object, pnts_2d_image = pttrn.get_data_for_pnp_solver()
    #save_pts_info(im_path, pnts_3d_object, pnts_2d_image)
    
    """ Step IV - Estimate the marker's pose """
    valid, rvec_pred, tvec_pred, inliers = cv.solvePnPRansac(pnts_3d_object, pnts_2d_image, cam_matrix, dist_coeff, None, None, False, 1000, 3.0, 0.9999, None, cv.SOLVEPNP_SQPNP)
    if not valid:
        return None
    #     raise Exception("PnP solver failed. ")
    
    if debug_im_path_stem is not None:
        features_image = draw_detected_and_projected_features(rvec_pred, tvec_pred, cam_matrix, dist_coeff, pttrn, im)
        cv.imwrite(os.path.join(debug_im_path_stem, "features_image.jpg"), features_image)

    show_reproj_error = False #True
    if show_reproj_error:
        reproj_error = get_reprojection_error(pnts_3d_object, rvec_pred, tvec_pred, inliers, cam_matrix, dist_coeff, pnts_2d_image, show_reproj_error, im)
    rmat_pred, _ = cv.Rodrigues(rvec_pred)
    tvec_pred = tvec_pred * 0.001 # Change [mm] to [m]
    
    offset = np.array([[-0.017], [0.0], [0.0]])  # Bottom of cylinder to TCP
    offset = np.dot(rmat_pred, offset)
    tvec_pred += offset
    transf = np.concatenate((rmat_pred, tvec_pred), axis = 1)
    transf = np.vstack((transf, [0., 0., 0., 1.])) # Making it homogeneous

    # Draw axis
    if debug_im_path_stem is not None:
        axis_image = show_axis(im, transf, rvec_pred, tvec_pred, cam_matrix, dist_coeff, 0.005)
        cv.imwrite(os.path.join(debug_im_path_stem, "axis_image.jpg"), axis_image)

    # Save solution
    # save_pose(img_format, im_path, transf)
    return transf
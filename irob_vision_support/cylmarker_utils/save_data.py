from cylmarker_utils import load_data
import os

from distutils.util import strtobool # for query_yes_no()
import yaml # for save_new_pttrn()


def query_yes_no(question):
    # ref: https://stackoverflow.com/questions/3041986/apt-command-line-interface-like-yes-no-input
    print('{} [y/n]'.format(question))
    while True:
        try:
            return strtobool(input().lower())
        except ValueError:
            print('Please respond with \'y\' or \'n\'.\n')


def check_and_warn_if_files_will_be_replaced(data_dir):
    file_pattern = load_data.get_path_pattern(data_dir)
    file_marker_img  = load_data.get_path_marker_img(data_dir)
    file_marker_data  = load_data.get_path_marker_data(data_dir)
    if os.path.isfile(file_pattern) or os.path.isfile(file_marker_img) or os.path.isfile(file_marker_data):
        message = ('Warning, the current marker/pattern in {} will be DELETED\n'
                   'and replaced by a new one! Do you wish to continue?'.format(data_dir))
        user_wants_to_continue = query_yes_no(message)
        if not user_wants_to_continue:
            print('Then, please choose another path for the new marker'
                  ', using `python --path new_path --task m`')
            exit()


def delete_file_if_exists(file_path):
    if os.path.isfile(file_path):
        os.remove(file_path)


def save_new_pttrn(data_dir, new_pttrn):
    file_pattern = load_data.get_path_pattern(data_dir)
    delete_file_if_exists(file_pattern)
    data_pattrn = {}
    for sqnc in new_pttrn.list_sqnc:
        sqnc_id = sqnc.sqnc_id
        sqnc_data = {}
        code, kpt_ids = sqnc.get_code_and_kpt_ids()
        sqnc_data[sqnc.NAME_CODE] = code
        sqnc_data[sqnc.NAME_KPT_IDS] = kpt_ids
        data_pattrn[sqnc_id] = sqnc_data
    # Save pattern data
    with open(file_pattern, 'w') as f:
        #data = yaml.dump(data_pattrn, f, default_flow_style=None) # horizontal alignment instead of vertical
        yaml.dump(data_pattrn, f, sort_keys=True) # vertical is more intuitive, since the sequences are columns in the marker


def save_new_marker(data_dir, new_marker, new_pttrn):
    # Save marker image file (the `svgwrite` library automatically replaces any old file)
    new_marker.save()
    # Save marker data
    file_marker_data = load_data.get_path_marker_data(data_dir)
    delete_file_if_exists(file_marker_data)
    data_marker = {}

    for sqnc in new_pttrn.list_sqnc:
        sqnc_id = sqnc.sqnc_id
        sqnc_data = {}
        for kpt in sqnc.list_kpts:
            kpt_id = kpt.kpt_id
            kpt_data = {}
            uv, xyz = kpt.get_centre_info()
            kpt_data[kpt.NAME_CNT_UV] = uv
            kpt_data[kpt.NAME_CNT_XYZ] = xyz
            kpt_data_corners = {}
            for corner_ind, corner_xyz in enumerate(kpt.xyz_corners):
                kpt_data_corners[corner_ind] = corner_xyz
            kpt_data[kpt.NAME_CRN_XYZ] = kpt_data_corners
            sqnc_data[kpt_id] = kpt_data
        data_marker[sqnc_id] = sqnc_data

    # Save in yaml file
    with open(file_marker_data, 'w') as f:
        yaml.dump(data_marker, f, default_flow_style=None)


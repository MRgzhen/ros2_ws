import numpy as np
from scipy.spatial.transform import Rotation as R


def matrix(rvec, tvec):
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(np.ravel(rvec)).as_matrix()
    T[:3, 3] = np.ravel(tvec)
    return T


def obj2base(T_base_cam, T_cam_obj):
    T_base_obj = T_base_cam @ T_cam_obj
    return T_base_obj


def extract(T):
    xyz = T[:3, 3]
    mat = T[:3, :3]
    quat = R.from_matrix(mat).as_quat()
    return xyz, quat

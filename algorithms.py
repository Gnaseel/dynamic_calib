import numpy as np
import odom_set
import math
from scipy import interpolate
from Bezier import Bezier
from numpy.random import default_rng

RNG = default_rng(23452345)

def getNoise(mat, rot_off, trans_off, noise):
    """
    Returns a 4*4 noise matrix
    INPUTS:
        mat         4*4 pos matrix
        rot_off     rotation offset (rad)
        trans_off   trans offset (m)
        noise       distribution of noise (normal, uniform)
    OUTPUTS:
        r_mat       4*4 noise matrix
    """

    noise_rpy = getattr(RNG,noise)(-rot_off, rot_off,3)
    noise_xyz = getattr(RNG,noise)(-trans_off, trans_off,3)

    noise_rotation_mat = odom_set.quaternion_rotation_matrix(odom_set.get_quaternion_from_euler(*noise_rpy))
    re_mat = np.eye(4)
    re_mat[:3,:3] = noise_rotation_mat
    re_mat[3,:3] += noise_xyz

    return re_mat

def getIntersectionFromPoints(pos1, pos2):
    """
    Returns a intersection point about input
    INPUTS:
        pos1, pos2  coordnate (x,y)
    OUTPUTS:
        px, py      intersection coordnate
    """

    pos1_yaw = odom_set.yaw2xy(pos1)[2]
    pos2_yaw = odom_set.yaw2xy(pos2)[2]
    print(f"POS1 {pos1}")
    print(f"POS2 {pos2}")
    print(f"YAW {pos1_yaw} {pos2_yaw}")

    b  = -1
    a1 = math.tan(pos1_yaw)
    c1 = pos1.y-pos1.x*a1
    a2 = math.tan(pos2_yaw)
    c2 = pos2.y-pos2.x*a2
    # print(f"EQ {a1} {c1}")
    # print(f"EQ {a2} {c2}")
    px = (c2-c1)/(a1-a2+0.000001)
    py = a1*(c2-c1)/(a1-a2+0.000001)+c1
    print(f"REASULT {px} {py}")
    return px, py

def getCameraTrajFromLiDAR(lidar_traj, calib_mat):
    """
    Return camera trajectory from lidar trajectory and calibration matrix
    INPUTS:
        lidar_traj      trajectory of lidar  [pos1, pos2, pos3...]
        calib_mat       4*4 calibration matrix between cam&lidar
    OUTPUTS:
        camera_traj     trajectory of camera [pos1, pos2, pos3...]
    """

    camera_traj = []
    for lidar_pos in lidar_traj:
        # pos = odom_set.pos(lidar_pos.x, lidar_pos.y)
        lidar_mat = odom_set.pos_2_TFMat(lidar_pos)
        cam_mat = odom_set.pcTransform(lidar_mat, calib_mat)
        
        new_pos = odom_set.TFMat_2_pos(cam_mat)
        # camera_traj.append([cam_mat[3,0], cam_mat[3,1]])
        camera_traj.append(new_pos)

        # print("-------------------")
        # print(lidar_pos)
        # print(camera_traj[-1])

    # camera_traj = np.array(camera_traj)
    return camera_traj

def getTrajFromOdom(pos1, pos2):
    """
    Returns trajectory from 2 odom vectors
    INPUTS:
        pos         odom vector
    OUTPUTS:
        pos_list    trajectory with bezier curve [pos1, pos2, pos3....]
    """

    dist = math.sqrt(math.pow(pos2.x-pos1.x, 2) + math.pow(pos2.y-pos1.y, 2))
    pos2_mat = odom_set.pos_2_TFMat(pos2)
    pred_delta = odom_set.pos_2_TFMat(odom_set.pos(dist, 0, 0, 1, 0, 0, 0))
    pos3_mat = odom_set.pcTransform(pos2_mat, pred_delta)
    pos3 = odom_set.TFMat_2_pos(pos3_mat)
    nx, ny = getIntersectionFromPoints(pos1, pos2)
    pos_inter = odom_set.pos(nx, ny)
    # pos2.x = nx
    # pos2.y = ny
    # print(pos1)
    # print(pos2)
    # print(pos3)
    # x = [pos1.x, pos2.x, pos3.x]
    # y = [pos1.y, pos2.y, pos3.y]
    # sp = interpolate.interp1d(x,y,kind='linear')
    # new_x = np.linspace(x[0], x[-1], 15)
    # new_y = sp(new_x)

    t_points = np.arange(0, 1.1, 0.0001) #................................. Creates an iterable list from 0 to 1.
    points1 = np.array([[pos1.x, pos1.y], [pos_inter.x, pos_inter.y], [pos2.x, pos2.y] ]) #.... Creates an array of coordinates.
    curve = Bezier.Curve(t_points, points1) #......................... Returns an array of coordinates.

    pos_list = []
    pos_list.append(pos1)
    for i in range(1    , len(curve)-3):
        # print(f'{curve[i]}')
        yaw = math.atan2((curve[i+3][1]-curve[i][1]) , (curve[i+3][0]-curve[i][0]+0.00001))
        quat = odom_set.get_quaternion_from_euler(0,0,yaw)
        # print(f"YAW {yaw}")
        pos = odom_set.pos(curve[i][0], curve[i][1], 0,  *quat)
        pos_list.append(pos)
    # print(f"CURVE {curve}")
    pos_list.append(pos2)
    return pos_list
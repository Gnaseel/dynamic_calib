import numpy as np
import odom_set
import math
from scipy import interpolate
from Bezier import Bezier

def getCameraTrajFromLiDAR(lidar_traj, calib_mat):
    camera_traj = []

    for lidar_pos in lidar_traj:
        print("-------------------")
        pos = odom_set.pos(lidar_pos[0], lidar_pos[1])
        lidar_mat = odom_set.pos_2_TFMat(pos)
        cam_mat = odom_set.pcTransform(lidar_mat, calib_mat)
        # lidar_pos[0] = cam_mat[3,0]
        # lidar_pos[1] = cam_mat[3,1]
        camera_traj.append([cam_mat[3,0], cam_mat[3,1]])

        print(lidar_pos)
        print(camera_traj[-1])

    camera_traj = np.array(camera_traj)
    return camera_traj

def getTrajFromOdom(pos1, pos2):
    print("HERE")
    dist = math.sqrt(math.pow(pos2.x-pos1.x, 2) + math.pow(pos2.y-pos1.y, 2))
    pos2_mat = odom_set.pos_2_TFMat(pos2)
    pred_delta = odom_set.pos_2_TFMat(odom_set.pos(dist, 0, 0, 1, 0, 0, 0))
    pos3_mat = odom_set.pcTransform(pos2_mat, pred_delta)
    pos3 = odom_set.TFMat_2_pos(pos3_mat)

    # print(pos1)
    # print(pos2)
    # print(pos3)
    # x = [pos1.x, pos2.x, pos3.x]
    # y = [pos1.y, pos2.y, pos3.y]
    # sp = interpolate.interp1d(x,y,kind='linear')
    # new_x = np.linspace(x[0], x[-1], 15)
    # new_y = sp(new_x)

    t_points = np.arange(0, 1, 0.01) #................................. Creates an iterable list from 0 to 1.
    points1 = np.array([[pos1.x, pos1.y], [pos2.x, pos2.y], [pos3.x, pos3.y] ]) #.... Creates an array of coordinates.
    curve = Bezier.Curve(t_points, points1) #......................... Returns an array of coordinates.
    # print(f"CURVE {curve}")
    return curve
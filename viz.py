import odom_set
import matplotlib.pyplot as plt
import math
import numpy as np
def yaw2xy(odom):
    return odom_set.euler_from_quaternion(odom.ox, odom.oy, odom.oz, odom.ow) #RPY

def draw_quiver(mat, color='g'):
    pos = odom_set.TFMat_2_pos(mat)
    yaw = yaw2xy(pos)[2]
    plt.quiver(pos.x, pos.y, math.cos(yaw)*30, math.sin(yaw)*30,color=color)

def viz_odom(quiver_list = [], cfg = None, liDAR_traj = None, camera_traj = None):


    calib_mat = odom_set.pos_2_TFMat(cfg.CalibExtrinsic)
    # print(f"CALIB MAT {calib_mat}")
    # print(f"CALIB POS {cfg.CalibExtrinsic}")

    for lidar_pos in quiver_list:
        # yaw = yaw2xy(lidar_pos)[2]
        # plt.quiver(lidar_pos.x, lidar_pos.y, math.cos(yaw)*30, math.sin(yaw)*30)
        
        liDAR_mat = odom_set.pos_2_TFMat(lidar_pos)
        camera_mat = odom_set.pcTransform(liDAR_mat, calib_mat)
        draw_quiver(liDAR_mat,'b')
        draw_quiver(camera_mat,'r')

        # print(f"LiDAR POS {lidar_pos}")
        # print("------------------------------------------")
        # print(f"LiDAR R MAT {liDAR_R_mat}")
        # print(f"LiDAR MAT {liDAR_mat}")
        # print(f"TRANSE MAT {camera_mat_t}")
    plt.plot( liDAR_traj[:, 0], liDAR_traj[:, 1])
    plt.plot( camera_traj[:, 0], camera_traj[:, 1])
    # plt.plot(liDAR_traj[0], liDAR_traj[1])
    plt.xticks([ i for i in range(10)])
    plt.yticks([ i for i in range(10)])
    plt.show()
    return
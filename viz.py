import odom_set
import matplotlib.pyplot as plt
import math
import numpy as np
import algorithms


def draw_quiver(mat, color='g'):
    pos = odom_set.TFMat_2_pos(mat)
    yaw = odom_set.yaw2xy(pos)[2]
    plt.quiver(pos.x, pos.y, math.cos(yaw)/500, math.sin(yaw)/500,color=color,  width=0.009, scale=0.095)

def draw_traj(pos_list):
    x_list = []
    y_list = []
    for pos in pos_list:
        x_list.append(pos.x)
        y_list.append(pos.y)
    x_list = np.array(x_list)
    y_list = np.array(y_list)
    plt.plot( x_list, y_list)
    return
    
def viz_odom(quiver_list = [], cfg = None, liDAR_traj = None, camera_traj = None):
    calib_mat = odom_set.pos_2_TFMat(cfg.CalibExtrinsic)

    # Draw LiDAR Camera trajectory (Quiver)
    i=0
    for lidar_pos in liDAR_traj:
        i+=1
        if i%350!=0:
            continue
        liDAR_mat = odom_set.pos_2_TFMat(lidar_pos)
        camera_mat = odom_set.pcTransform(liDAR_mat, calib_mat)
        # draw_quiver(liDAR_mat,'g')
        # draw_quiver(camera_mat,'b')

        # noise_mat = algorithms.getNoise(camera_mat, odom_set.deg2rad(5), 0.1, "normal")
        # draw_quiver(noise_mat,'y')


    # Draw LiDAR Camera trajectory (Line)
    draw_traj(liDAR_traj)
    draw_traj(camera_traj)


    # Draw LiDAR Camera Start poisition

    for lidar_pos in quiver_list:
        # yaw = yaw2xy(lidar_pos)[2]
        # plt.quiver(lidar_pos.x, lidar_pos.y, math.cos(yaw)*30, math.sin(yaw)*30)
        
        liDAR_mat = odom_set.pos_2_TFMat(lidar_pos)
        camera_mat = odom_set.pcTransform(liDAR_mat, calib_mat)
        draw_quiver(liDAR_mat,'g')
        draw_quiver(camera_mat,'b')

    
    # Draw Noise
    for i in range(10):
        liDAR_mat = odom_set.pos_2_TFMat(quiver_list[1])
        camera_mat = odom_set.pcTransform(liDAR_mat, calib_mat)       
        noise_mat = algorithms.getNoise(camera_mat, odom_set.deg2rad(20), 1.5, "uniform")
        draw_quiver(noise_mat,'y')

    # plt.plot( liDAR_traj[:, 0], liDAR_traj[:, 1])
    # plt.plot( camera_traj[:, 0], camera_traj[:, 1])
    # plt.plot(liDAR_traj[0], liDAR_traj[1])
    plt.xticks([ i for i in range(10)])
    plt.yticks([ i for i in range(10)])
    plt.rcParams["figure.figsize"] = (120,3)
    plt.show()
    return
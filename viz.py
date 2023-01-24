import odom_set
import matplotlib.pyplot as plt
import math
import numpy as np
import algorithms
from sklearn.neighbors import KernelDensity
from scipy import integrate
from scipy.stats import entropy
def draw_quiver(mat, color='g'):
    pos = odom_set.TFMat_2_pos(mat)
    yaw = odom_set.getRPY_fromPos(pos)[2]
    # plt.quiver(pos.x, pos.y, math.cos(yaw)/70, math.sin(yaw)/70,color=color,  width=0.0009, scale=1)
    plt.quiver(pos.x, pos.y, math.cos(yaw)/500, math.sin(yaw)/500,color=color,  width=0.009, scale=0.095)

def draw_traj(pos_list):
    x_list = []
    y_list = []
    for pos in pos_list:
        x_list.append(pos.x)
        y_list.append(pos.y)
    x_list = np.array(x_list)
    y_list = np.array(y_list)
    plt.plot( x_list, y_list, color='g')

def noise_viz(noise_list):
    """
    Visualizes distribution of noise with matplot
    INPUTS:
        noise_mat_list   list of (4*4) noise matrix
    """
    name_list = ["roll", "pitch", "yaw", "x", "y", "z"]

    entropy_list = []
    std_list = []
    for i in range(6):
        plt.subplot(2,3,i+1)
        a = np.array(noise_list[i]).reshape(-1, 1)

        kde = KernelDensity(kernel='gaussian', bandwidth=0.01).fit(a)
        if i<3:
            width = 1.0
        else:
            width = 2
        s = np.linspace(-width,width, 1000)
        e = kde.score_samples(s.reshape(-1,1))

        valmin = np.min(a)
        valmax = np.max(a)
        plt.fill(s, np.exp(e), fc="#AAAAFF")
        

        hist, bins = np.histogram(a)
        # print(f"Hist {hist}")
        # print(f"Bins {bins}")
        hist = np.divide(hist, np.sum(hist))
        H = entropy(hist)
        std = np.std(a)
        entropy_list.append(H)
        std_list.append(std)
        print(f"    Entropy {H}")
        print(f"    Std {std}")
        plt.ylim(0,12)
        plt.text(0, -2, name_list[i], size=20, ha='center')

    print(f"    Entropy mean {sum(entropy_list)/len(entropy_list)}")
    print(f"    STV mean {sum(std_list)/len(std_list)}")
    return

def viz_odom(quiver_list = [], cfg = None, liDAR_traj = None, camera_traj = None):
    """
    Main viz
    """
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
    # draw_traj(liDAR_traj)
    # draw_traj(camera_traj)


    # Draw LiDAR Camera Start poisition

    for lidar_pos in quiver_list:
        # yaw = getRPY_fromPos(lidar_pos)[2]
        # plt.quiver(lidar_pos.x, lidar_pos.y, math.cos(yaw)*30, math.sin(yaw)*30)
        
        liDAR_mat = odom_set.pos_2_TFMat(lidar_pos)
        camera_mat = odom_set.pcTransform(liDAR_mat, calib_mat)
        # draw_quiver(liDAR_mat,'g')
        # draw_quiver(camera_mat,'b')

    noise_mat_list = []
    # Draw Noise
    for i in range(1000):
        liDAR_mat = odom_set.pos_2_TFMat(quiver_list[1])
        camera_mat = odom_set.pcTransform(liDAR_mat, calib_mat)       
        noise_mat = algorithms.getNoise(camera_mat, odom_set.deg2rad(20), 1.5, "uniform")
        
        noise_mat_list.append(noise_mat)


        re_mat = np.eye(4)
        re_mat[:3,:3] = noise_mat[:3,:3]
        re_mat = np.matmul(re_mat, camera_mat)
        re_mat[3,:3] += noise_mat[3,:3]
        # draw_quiver(re_mat,'y')

    noise_viz(noise_mat_list)
    plt.show()
    return
    # plt.plot( liDAR_traj[:, 0], liDAR_traj[:, 1])
    # plt.plot( camera_traj[:, 0], camera_traj[:, 1])
    # plt.plot(liDAR_traj[0], liDAR_traj[1])
    plt.xticks([ i for i in range(10)])
    plt.yticks([ i for i in range(10)])
    plt.rcParams["figure.figsize"] = (120,3)
    plt.show()
    return

def viz_trajectories(traj_list):

    for traj in traj_list:
        draw_traj(traj)
        for odom in traj:
            mat = odom_set.pos_2_TFMat(odom)
            draw_quiver(mat)
            
    # draw_traj(camera_traj)
    return

def plot_show():
    plt.show()
    
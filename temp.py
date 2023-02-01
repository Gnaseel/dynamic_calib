import odom_set
import algorithms
import viz
import numpy as np
cfg = odom_set.cfg()
calib_mat = odom_set.pos_2_TFMat(cfg.CalibExtrinsic)

def get_kitti_noise(show_plot = False, odom_file = "/workspace/data/KITTI_odom/0926_0001_odom_list.txt"):
# def get_kitti_noise(show_plot = False, odom_file = "/workspace/data/KITTI_odom/0926_0005_odom_list.txt"):
# def get_kitti_noise(show_plot = False, odom_file = "/workspace/data/KITTI_odom/0926_0070_odom_list.txt"):
# def get_kitti_noise(show_plot = False, odom_file = "/workspace/data/KITTI_odom/0926_0113_odom_list.txt"):
    """
    INPUTS:
    OUTPUTS:
        noise_mat_list  : list of noise matrix | format : [[r], [p], [y], [x]...]
    """
    odom_list = algorithms.readOdomfile(odom_file)
    # odom_list = algorithms.readOdomfile("/workspace/data_shl/kitti_odom_path/0926_0005_odom_list.txt")

    # odom_list = algorithms.readOdomfile("/home/shl/Project/Dynamic_calib/data/0926_0001_odom_list.txt")
    traj_list = []
    start = 0
    end = 100
    # end = len(odom_list)-1
    # end = len(odom_list)-1
    print(f"Len of odom {len(odom_list)}")

    # Get LiDAR trajectory form odom
    step = 1
    for i in range(start, end, step):
        # print(f"{i} / {end}")
        traj = algorithms.getTrajFromOdom(odom_list[i], odom_list[i+1])
        traj_list.append(traj)
    
    # Get noise matrix list
    noise_mat_list = algorithms.getNoiseMat_from_trajList(traj_list)
    print(f"LEN noise {len(noise_mat_list)}")

    if show_plot:
        viz.noise_viz(noise_mat_list)
        # viz.viz_trajectories(traj_list)
        viz.plot_show()
    else:
        pdf = algorithms.getNoiseDis(noise_mat_list)
    return noise_mat_list

def viz_noise_from_gt_file(path="/workspace/data/KITTI/train/2011_09_26/2011_09_26_drive_0001_sync/gt_realistic.txt"):
# def viz_noise_from_gt_file(path="/workspace/data/KITTI/train/2011_09_26/2011_09_26_drive_0001_sync/gt_E_1.txt"):
    f = open(path)
    lines = f.readlines()

    noise_list = [[] for i in range(6)] # list of rpyxyz

    for line in lines:
        word = line.split(',')[1:]
        roll, pitch, yaw, x, y, z = float(word[0]), float(word[1]), float(word[2]), float(word[3]), float(word[4]), float(word[5]) 
        noise_list[0].append(roll)
        noise_list[1].append(pitch)
        noise_list[2].append(yaw)
        noise_list[3].append(x)
        noise_list[4].append(y)
        noise_list[5].append(z)
        # noise_mat = np.zeros((4,4))
        # # print(word)
        # qw,qx,qy,qz = odom_set.get_quaternion_from_euler(roll, pitch, yaw)
        # noise_mat[:3,:3] = odom_set.quaternion_rotation_matrix([qw,qx,qy,qz])
        # noise_mat[3,:3] = [x,y,z]
        # # print(noise_mat)

    viz.noise_viz(noise_list)
    viz.plot_show()


    return
print("HIHI")

if __name__=="__main__":
    # quiver_list = [odom_set.getSamplePos(0), odom_set.getSamplePos(1)]
    # viz_kitti_noise()
    # viz_noise_from_gt_file()
    get_kitti_noise(show_plot=True)



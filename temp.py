import odom_set
import algorithms
import viz
import numpy as np
cfg = odom_set.cfg()
calib_mat = odom_set.pos_2_TFMat(cfg.CalibExtrinsic)

def get_kitti_noise(show_plot = False):
    # odom_list = algorithms.readOdomfile("/workspace/data_shl/kitti_odom_path/0926_0001_odom_list.txt")
    odom_list = algorithms.readOdomfile("/workspace/data_shl/kitti_odom_path/0926_0005_odom_list.txt")

    # odom_list = algorithms.readOdomfile("/home/shl/Project/Dynamic_calib/data/0926_0001_odom_list.txt")
    traj_list = []
    start = 0
    # end = 100
    end = len(odom_list)-1
    # end = len(odom_list)-1
    print(f"Len {len(odom_list)}")

    # Get LiDAR trajectory form odom
    step = 1
    for i in range(start, end, step):
        # print(f"{i} / {end}")
        traj = algorithms.getTrajFromOdom(odom_list[i], odom_list[i+1])
        traj_list.append(traj)
    
    # Get noise matrix list
    noise_mat_list = algorithms.getNoiseMat_from_trajList(traj_list)

    if show_plot:
        viz.noise_viz(noise_mat_list)
        # viz.viz_trajectories(traj_list)
        viz.plot_show()
    else:
        pdf = algorithms.getNoiseDis(noise_mat_list)
    return noise_mat_list

print("HIHI")

if __name__=="__main__":
    quiver_list = [odom_set.getSamplePos(0), odom_set.getSamplePos(1)]
    get_kitti_noise(show_plot=True)
    # viz_kitti_noise()



import odom_set
import algorithms
import viz
import numpy as np
cfg = odom_set.cfg()
calib_mat = odom_set.pos_2_TFMat(cfg.CalibExtrinsic)

def get_kitti_noise():
    odom_list = algorithms.readOdomfile("/home/shl/Project/Dynamic_calib/data/0926_0005_odom_list.txt")
    # odom_list = algorithms.readOdomfile("/home/shl/Project/Dynamic_calib/data/0926_0001_odom_list.txt")
    traj_list = []
    start = 0
    # end = 100
    end = len(odom_list)-90
    # end = len(odom_list)-1
    print(f"Len {len(odom_list)}")

    # Get LiDAR trajectory form odom
    for i in range(start, end):
        print(f"{i} / {end}")
        traj = algorithms.getTrajFromOdom(odom_list[i], odom_list[i+1])
        traj_list.append(traj)
    
    # Get noise matrix list
    noise_mat_list = []
    for traj in traj_list:
        ori_lidar_mat = odom_set.pos_2_TFMat(traj[int(len(traj)/2)])
        ori_cam_mat = odom_set.pcTransform(ori_lidar_mat, calib_mat)
        # print(f"Ori cam {ori_cam_mat[3]}")
        for lidar_pos in traj:
            lidar_mat = odom_set.pos_2_TFMat(lidar_pos)
            # print(f" LiDAR {lidar_pos}")
            # print(f"LiDAR POSE {lidar_pos}")
            camera_mat = odom_set.pcTransform(lidar_mat, calib_mat)
            noise_mat = algorithms.getNoise(camera_mat, odom_set.deg2rad(5), 0.05, "normal")
            # noise_mat = algorithms.getNoise(camera_mat, odom_set.deg2rad(5), 0.1, "uniform")
            abs_noise_mat = odom_set.pcTransform(camera_mat, noise_mat)

            abs_noise_mat[3,:3] -= ori_cam_mat[3,:3]
            reverse_cam_mat = np.eye(4)
            reverse_cam_mat[:3,:3] = np.transpose(ori_cam_mat[:3,:3])

            abs_noise_mat = odom_set.pcTransform(abs_noise_mat, reverse_cam_mat)
            abs_noise_pos = odom_set.TFMat_2_pos(abs_noise_mat)
            r, p, y = odom_set.getRPY_fromPos(abs_noise_pos)
            cam_pos = odom_set.TFMat_2_pos(camera_mat)
            cr, cp, cy = odom_set.getRPY_fromPos(cam_pos)

            # print(f"    Subscr x {abs_noise_pos.x} y {abs_noise_pos.y} z {abs_noise_pos.z}")
            # print(f"    Subscr r {r} p {p} y {y}")

            # print(f"CAM {cam_pos.x} {cam_pos.y}")
            # print()

            # Uniform noise
            abs_noise_mat = algorithms.getNoise(camera_mat, odom_set.deg2rad(20), 1.5, "uniform")
            noise_mat_list.append(abs_noise_mat)

        # noise_mat = algorithms.getNoise(camera_mat, odom_set.deg2rad(5), 0.1, "normal")
    viz.noise_viz(noise_mat_list)
    # viz.viz_trajectories(traj_list)
    viz.plot_show()
    return

print("HIHI")

# quiver_list = [odom_set.getSamplePos(0), odom_set.getSamplePos(1), odom_set.getSamplePos(2)]



# liDAR_traj = algorithms.getTrajFromOdom(quiver_list[0], quiver_list[1])
# camera_traj = algorithms.getCameraTrajFromLiDAR(liDAR_traj, cfg.CalibExtrinsic_matrix)

# viz.viz_odom(quiver_list, cfg, liDAR_traj, camera_traj)

if __name__=="__main__":
    quiver_list = [odom_set.getSamplePos(0), odom_set.getSamplePos(1)]
    get_kitti_noise()


import odom_set
import algorithms
import viz


print("HIHI")

# quiver_list = [odom_set.getSamplePos(0), odom_set.getSamplePos(1), odom_set.getSamplePos(2)]
quiver_list = [odom_set.getSamplePos(0), odom_set.getSamplePos(1)]
cfg = odom_set.cfg()


liDAR_traj = algorithms.getTrajFromOdom(quiver_list[0], quiver_list[1])
camera_traj = algorithms.getCameraTrajFromLiDAR(liDAR_traj, cfg.CalibExtrinsic_matrix)

viz.viz_odom(quiver_list, cfg, liDAR_traj, camera_traj)
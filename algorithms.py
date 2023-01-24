import numpy as np
import odom_set
import math
from scipy import interpolate
from Bezier import Bezier
from numpy.random import default_rng
from sklearn.neighbors import KernelDensity
from scipy.stats import entropy

import viz
RNG = default_rng(23452345)

def getNoiseDis(noise_list):
    """
    get noise distribution from noise matrix list
    INPUTS:
        list of noise         [ noise[r], noise1[p], noise[y]... xyz]
    OUTPUTS:
        noise distribution list     rpyxyz (list of distribution) - probability density function (scikit learn)
    """
    pdf_list = []
    for i in range(6):
        # plt.subplot(2,3,i+1)
        a = np.array(noise_list[i]).reshape(-1, 1)

        kde = KernelDensity(kernel='gaussian', bandwidth=0.01).fit(a)
        # if i<3:
        #     width = 1.0
        # else:
        #     width = 2
        # s = np.linspace(-width,width, 1000)
        # e = kde.score_samples(s.reshape(-1,1))
        # hist, bins = np.histogram(a)
        # print(f"Hist {hist}")
        # print(f"Bins {bins}")
        # hist = np.divide(hist, np.sum(hist))
        pdf_list.append(kde)
    return pdf_list

def getNoiseMat_from_trajList(traj_list):
    noise_mat_list=[]
    cfg = odom_set.cfg()
    calib_mat = odom_set.pos_2_TFMat(cfg.CalibExtrinsic)

    for traj in traj_list:
        ori_lidar_mat = odom_set.pos_2_TFMat(traj[int(len(traj)/2)])
        ori_cam_mat = odom_set.pcTransform(ori_lidar_mat, calib_mat)
        # print(f"Ori cam {ori_cam_mat[3]}")
        for lidar_pos in traj:
            lidar_mat = odom_set.pos_2_TFMat(lidar_pos)
            # print(f" LiDAR {lidar_pos}")
            # print(f"LiDAR POSE {lidar_pos}")
            camera_mat = odom_set.pcTransform(lidar_mat, calib_mat)
            noise_mat = getNoise(camera_mat, odom_set.deg2rad(5), 0.05, "normal")
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

            # Uniform noise
            abs_noise_mat = getNoise(camera_mat, odom_set.deg2rad(20), 1.5, "uniform")
            noise_mat_list.append(abs_noise_mat)

    noise_list = [[] for i in range(6)] # list of rpyxyz
    for noise_mat in noise_mat_list:
        nroll, npitch, nyaw = odom_set.euler_from_quaternion(*odom_set.rotationMatrix_2_Quaternion(noise_mat[:3,:3]))
        nx, ny, nz = noise_mat[3,:3]
        noise_list[0].append(nroll)
        noise_list[1].append(npitch)
        noise_list[2].append(nyaw)
        noise_list[3].append(nx)
        noise_list[4].append(ny)
        noise_list[5].append(nz)

    return noise_list

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
    # print(f"IN A z{re_mat[3]}")
    return re_mat

def getIntersectionFromPoints(pos1, pos2):
    """
    Returns a intersection point about input
    INPUTS:
        pos1, pos2  coordnate (x,y)
    OUTPUTS:
        px, py      intersection coordnate
    """

    pos1_yaw = odom_set.getRPY_fromPos(pos1)[2]
    pos2_yaw = odom_set.getRPY_fromPos(pos2)[2]
    # print(f"POS1 {pos1}")
    # print(f"POS2 {pos2}")
    # print(f"YAW {pos1_yaw} {pos2_yaw}")

    b  = -1
    a1 = math.tan(pos1_yaw)
    c1 = pos1.y-pos1.x*a1
    a2 = math.tan(pos2_yaw)
    c2 = pos2.y-pos2.x*a2
    # print(f"EQ {a1} {c1}")
    # print(f"EQ {a2} {c2}")
    px = (c2-c1)/(a1-a2+0.000001)
    py = a1*(c2-c1)/(a1-a2+0.000001)+c1
    # print(f"REASULT {px} {py}")
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
    # pred_delta = odom_set.pos_2_TFMat(odom_set.pos(dist, 0, 0, 1, 0, 0, 0))
    # INTERSECTION
    # nx, ny = getIntersectionFromPoints(pos1, pos2)
    # pos_inter = odom_set.pos(nx, ny)
    pos_inter = odom_set.pos((pos1.x+pos2.x)/2, (pos1.y+pos2.y)/2)

    # print(pos1)
    # print(pos_inter)
    # print(pos2)
    
    # viz.draw_quiver(odom_set.pos_2_TFMat(pos1))
    # viz.draw_quiver(odom_set.pos_2_TFMat(pos2))

    t_points = np.arange(-0.1, 1.1, 0.3) #................................. Creates an iterable list from 0 to 1.
    points1 = np.array([[pos1.x, pos1.y, pos1.z], [pos_inter.x, pos_inter.y, pos_inter.z], [pos2.x, pos2.y, pos2.z] ]) #.... Creates an array of coordinates.
    curve = Bezier.Curve(t_points, points1) #......................... Returns an array of coordinates.
    # print(curve)
    pos_list = []
    # pos_list.append(pos1)
    iter=5
    roll, pitch, yaw = odom_set.getRPY_fromPos(pos1)
    droll, dpitch, dyaw = (a_i - b_i for a_i, b_i in zip(odom_set.getRPY_fromPos(pos2) , odom_set.getRPY_fromPos(pos1)))
    droll, dpitch, dyaw = (droll+2*math.pi)%(2*math.pi), (dpitch+2*math.pi)%(2*math.pi), (dyaw+2*math.pi)%(2*math.pi)
    if droll>math.pi:
        droll -= 2*math.pi
    if dpitch>math.pi:
        dpitch -= 2*math.pi
    if dyaw>math.pi:
        dyaw -= 2*math.pi
    for i in range(iter+1):
        x = pos1.x + (pos2.x - pos1.x)*i/iter
        y = pos1.y + (pos2.y - pos1.y)*i/iter
        z = pos1.z + (pos2.z - pos1.z)*i/iter
        new_quat = odom_set.get_quaternion_from_euler(roll+droll*i/iter, pitch+dpitch*i/iter, yaw+dyaw*i/iter)
        # npos = odom_set.pos(x, y, z, pos1.ow, pos1.ox, pos1.oy, pos1.oz)
        npos = odom_set.pos(x, y, z, *new_quat)

        pos_list.append(npos)
    # print(f"LEN {len(pos_list)}")
    # return
    # for i in range(1    , len(curve)-3):
    #     roll = math.atan2((curve[i+3][2]-curve[i][2]) , (curve[i+3][1]-curve[i][1]+0.00001))
    #     pitch = math.atan2((curve[i+3][0]-curve[i][0]) , (curve[i+3][2]-curve[i][2]+0.00001))
    #     yaw = math.atan2((curve[i+3][1]-curve[i][1]) , (curve[i+3][0]-curve[i][0]+0.00001))
    #     quat = odom_set.get_quaternion_from_euler(roll, pitch ,yaw)
    #     # quat = odom_set.get_quaternion_from_euler(0, 0 ,yaw)
    #     # print(f" r {roll} p {pitch} y {yaw}")
    #     pos = odom_set.pos(curve[i][0], curve[i][1], curve[i][2],  *quat)
    #     pos_list.append(pos)
    # pos_list.append(pos2)
    return pos_list

def readOdomfile(filePath):
    """
    read odomfile 
    INPUTS:
        filePath    path of file (string)
                    file data format  ( (rotation)wxyz + (translation) xyz)\n
    OUTPUTS:
        pos_list    list of odom 
                    list data format    [pos1, pos2, pos3...]
                    
    """

    f = open(filePath)
    odom_list = f.readlines()
    pos_list= []
    for i, odom in enumerate(odom_list):
        data = odom.split(' ')
        for ii, v in enumerate(data):
            data[ii] = float(data[ii])
        pos = odom_set.pos(*data[4:], *data[:4])
        pos_list.append(pos)

    return pos_list

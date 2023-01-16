import numpy as np
import math
import copy

class pos:
    def __init__(self, x=0,y=0,z=0,ow=1, ox=0,oy=0,oz=0):
        self.x = x
        self.y = y
        self.z = z
        self.ox = ox
        self.oy = oy
        self.oz = oz
        self.ow = ow
        return
    def __str__(self):
        return f"x = {self.x} , y = {self.y} , z = {self.z} , ow = {self.ow} , ox = {self.ox} , oy = {self.oy} , oz = {self.oz}"

class cfg:
    def __init__(self):
        self.CalibExtrinsic = pos(2.0, 1.0, 0, *get_quaternion_from_euler(0,0,deg2rad(45)))
        self.CalibExtrinsic_matrix = pos_2_TFMat(self.CalibExtrinsic)
        return
    def __str__(self):
        return "SEX"

def TFMat_2_pos(mat):
    myPos = pos()
    q = rotationMatrix_2_Quaternion(mat[:3,:3])
    # print(f"Q = {q}")

    myPos.ow = q[0]
    myPos.ox = q[1]
    myPos.oy = q[2]
    myPos.oz = q[3]
    myPos.x = mat[3,0]
    myPos.y = mat[3,1]
    myPos.z = mat[3,2]
    return myPos

def pos_2_TFMat(pos):
    mat = np.zeros((4,4))
    quatMat = quaternion_rotation_matrix([ pos.ow, pos.ox, pos.oy, pos.oz])
    tranMat = np.array([pos.x, pos.y, pos.z, 1])
    mat[:3,:3] = quatMat
    mat[3] = tranMat
    return mat

def pcTransform(parent, child):
    parent_R_mat = np.zeros((4,4))
    parent_R_mat[:3,:3] = parent[:3,:3]
    parent_R_mat[3,3]=1
    child_mat_t = np.matmul(parent_R_mat, np.transpose(child[3]))
    child_mat = np.matmul(parent_R_mat, child)
    child_mat[3] = np.transpose(child_mat_t+ parent[3]) 
    return child_mat

def rotationMatrix_2_Quaternion(m):
    #q0 = qw
    t = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if(t > 0):
        t = np.sqrt(t + 1)
        q[3] = 0.5 * t
        t = 0.5/t
        q[0] = (m[2,1] - m[1,2]) * t
        q[1] = (m[0,2] - m[2,0]) * t
        q[2] = (m[1,0] - m[0,1]) * t

    else:
        i = 0
        if (m[1,1] > m[0,0]):
            i = 1
        if (m[2,2] > m[i,i]):
            i = 2
        j = (i+1)%3
        k = (j+1)%3

        t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (m[k,j] - m[j,k]) * t
        q[j] = (m[j,i] + m[i,j]) * t
        q[k] = (m[k,i] + m[i,k]) * t
    # print(f"BQ IS {q}")
    temp = q[3]
    q[3] = q[2]
    q[2] = q[1]
    q[1] = q[0]
    q[0] = temp
    # print(f"AQ IS {q}")
    return q

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
    # print(f"{q0} {q1} {q2} {q3}")
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def deg2rad(deg):
    return deg*3.141592/180

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return  qw, qx, qy, qz : The orientation in quaternion [w,x,y,z] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [ qw, qx, qy, qz]
 
def euler_from_quaternion(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return [roll_x, pitch_y, yaw_z] # in radians

def getOdomDelta(pos_prev, pos_curr):

    return

def getSamplePos(i):
    poseList=[]

    pos1 = pos(1,2,3,*get_quaternion_from_euler(0,0,deg2rad(0)))
    poseList.append(pos1)
    # pos1 = pos(10,2,7,*get_quaternion_from_euler(0,0,deg2rad(0)))
    # poseList.append(pos1)
    # pos1 = pos(4,5,7,*get_quaternion_from_euler(0,0,deg2rad(90)))
    # poseList.append(pos1)
    pos1 = pos(1.5,2.7,0,*get_quaternion_from_euler(0,0,deg2rad(65)))
    poseList.append(pos1)
    pos1 = pos(6,6,0,*get_quaternion_from_euler(0,0,deg2rad(45)))
    poseList.append(pos1)
    return poseList[i]

def getRPY_fromPos(pos):
    """
    return rpy from pos
    INPUTS:
        pos     pos (wxyz required)
    OUTPUTS
        rpy     [roll, pitch, yaw]  (rad)
    """
    return euler_from_quaternion(pos.ow, pos.ox, pos.oy, pos.oz) #RPY

    
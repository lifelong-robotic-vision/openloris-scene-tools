#!/usr/bin/env python2
import rosbag
import sys
import roslib
import rospy
import numpy as np
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseStamped, TransformStamped
from tf.transformations import quaternion_multiply,quaternion_conjugate
from rospy.rostime import Time
from tf2_msgs.msg import TFMessage
FLOAT_EPS = np.finfo(np.float).eps

##the translation and quaternion from d435i to marker
##note the quaternion is qx,qy,qz,qw
ts_m_d=[-0.0318876, 0.0490709,-0.00575465]
qt_m_d=[-0.707826, -0.0133337, -0.00506051, 0.706243]
##the translation and quaternion from marker to t265_imu
##note the quaternion is qx,qy,qz,qw
ts_imu_marker=[0.032653488512015247,-0.07086889526625034,0.02004296168173806]
qt_imu_marker=[0.08673214386231039,-0.022916885982446849,-0.04000986672047097,0.9951640880392164]
##translation and quaternion from t265 to imu
##note the quaternion is qx,qy,qz,qw
ts_imu_t265=[-0.001057399052113554,0.0020920342503451528,-0.0002740477123929344]
qt_imu_t265=[-0.6529740355381054,0.005953328667198988,-0.05821788685357764,0.7551159807878293]

T_marker_d435i=np.array([[0.999593, 0.0260238, -0.0116698, -0.0318876],
[0.011728, -0.00208632, 0.999929, 0.0490709],
[0.0259976, -0.999659, -0.00239068, -0.00575465],
[0, 0, 0, 1]])


def main():
    odom_frame_id = 'base_odom'
    odom_child_frame_id = 'base_link'
    mocap_topic = '/vrpn_client_node/RigidBody1/pose'

    infile = sys.argv[1] if len(sys.argv) > 1 else 'preprocessed.bag'
    outfile = 'processed.bag'

    with rosbag.Bag(infile) as inbag, rosbag.Bag(outfile, 'w') as outbag:
        print ("-------------------- Input: %s ------------------------------" % infile)
        print (inbag)
        for topic,msg,t in inbag.read_messages():
            if topic == mocap_topic:
                outbag.write(topic,msg,t)
                world_to_d435i(msg,t,outbag)
                world_to_t265(msg,t,outbag)
            else:
                outbag.write(topic,msg,t)
        outbag.close()
        outbag = rosbag.Bag(outfile)
        print ("------------------- Output: %s ------------------------------" % outfile)
        print (outbag)

def world_to_d435i(msg,t,outbag,frame_id="/map"):
                '''
                turn /vrpn_client_node/RigidBody1/pose topic to /groundtruth_d435i topic
                /vrpn_client_node/RigidBody1/pose message example:
                                header:
                                  seq: 38182
                                  stamp:
                                    secs: 8058
                                    nsecs:  24999000
                                  frame_id: "world"
                                pose:
                                  position:
                                    x: -2.22959446907
                                    y: -0.488971710205
                                    z: -0.164288565516
                                  orientation:
                                    x: -0.0124676739797
                                    y: 0.00673281447962
                                    z: 0.999864757061
                                    w: 0.00835528038442
                this function generate the transformation from d435i to world, which the view in world coordinate
                '''
                pose=PoseStamped()
                pose.header=msg.header
                pose.header.frame_id=frame_id
                '''
                the translation and quaternion of marker to world
                '''
                ts_w_m=msg.pose.position
                qt_w_m=msg.pose.orientation
                _ts_w_m=[ts_w_m.x,ts_w_m.y,ts_w_m.z]
                _qt_w_m=[qt_w_m.x,qt_w_m.y,qt_w_m.z,qt_w_m.w]

                R_world_marker=quat2mat(_qt_w_m)
                T_world_marker=getTransformation(R_world_marker,_ts_w_m)
                T_world_d435i=np.asmatrix(T_world_marker)*np.asmatrix(T_marker_d435i)
                q_world_d435i,t_world_d435i=getQuaternion_and_translation(np.asarray(T_world_d435i))
                ##print "from transform: "
                ##print q_world_d435i,t_world_d435i

                pose.pose.position=Point(*t_world_d435i)
                pose.pose.orientation=Quaternion(*q_world_d435i)
                outbag.write('/groundtruth_d435i',pose,t)


def world_to_t265(msg,t,outbag,frame_id="/map"):
                '''
                turn /vrpn_client_node/RigidBody1/pose topic to /groundtruth_d435i topic

                this function generate the transformation from d435i to world, which the view in world coordinate
                '''
                ##the transformation matrix from t265 to imu
                R_i_t=quat2mat(qt_imu_t265)
                T_i_t=getTransformation(R_i_t,ts_imu_t265)

                ##the transformation matrix from marker to imu
                R_i_m=quat2mat(qt_imu_marker)
                T_i_m=getTransformation(R_i_m,ts_imu_marker)

                T_m_i=np.linalg.inv(np.asmatrix(T_i_m))
                T_marker_t265=T_m_i*np.asmatrix(T_i_t)

                pose=PoseStamped()
                pose.header=msg.header
                pose.header.frame_id=frame_id
                '''
                the translation and quaternion of marker to world
                '''
                ts_w_m=msg.pose.position
                qt_w_m=msg.pose.orientation
                _ts_w_m=[ts_w_m.x,ts_w_m.y,ts_w_m.z]
                _qt_w_m=[qt_w_m.x,qt_w_m.y,qt_w_m.z,qt_w_m.w]

                R_w_m=quat2mat(_qt_w_m)
                T_w_m=getTransformation(R_w_m,_ts_w_m)
                '''
                after the composition,it's the translation and quaternnion from t265 to world
                '''
                T_world_t265=np.asmatrix(T_w_m)*T_marker_t265

                qt_world_t265,t_world_t265=getQuaternion_and_translation(np.asarray(T_world_t265))
                pose.pose.position=Point(*t_world_t265)
                pose.pose.orientation=Quaternion(*qt_world_t265)
                outbag.write('/groundtruth_t265',pose,t)


def getQuaternion_and_translation(TranM):
    '''
        input: a 4x4 transformation matrix
        output:a quaternion and a translation
    '''
    R,t=get_Rt_qt(TranM)
    quat=quaternion_from_matrix(R)
    return quat,t

def getTransformation(R,t):
    ts=np.eye(4)
    for i in range(3):
        ts[i][3]=t[i]
        for j in range(3):
            ts[i][j]=R[i][j]
    return ts

def get_Rt_qt(T):
    R=np.eye(3)
    t=[]
    for i in range(3):
        t.append(T[i][3])
        for j in range(3):
            R[i][j]=T[i][j]
    return R,t

def quat2mat(q):
    ''' Calculate rotation matrix corresponding to quaternion

    Parameters
    ----------
    q : 4 element array-like

    Returns
    -------
    M : (3,3) array
      Rotation matrix corresponding to input quaternion *q*

    Notes
    -----
    Rotation matrix applies to column vectors, and is applied to the
    left of coordinate vectors.  The algorithm here allows non-unit
    quaternions.

    References
    ----------
    Algorithm from
    http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion

    Examples
    --------
    >>> import numpy as np
    >>> M = quat2mat([0, 0, 0, 1]) # Identity quaternion
    >>> np.allclose(M, np.eye(3))
    True
    >>> M = quat2mat([0, 0, 1, 0]) # 180 degree rotn around axis 0
    >>> np.allclose(M, np.diag([1, -1, -1]))
    True
    '''
    x, y, z, w = q
    Nq = w*w + x*x + y*y + z*z
    if Nq < FLOAT_EPS:
        return np.eye(3)
    s = 2.0/Nq
    X = x*s
    Y = y*s
    Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z
    return np.array(
           [[ 1.0-(yY+zZ), xY-wZ, xZ+wY ],
            [ xY+wZ, 1.0-(xX+zZ), yZ-wX ],
            [ xZ-wY, yZ+wX, 1.0-(xX+yY) ]])


def quaternion_from_matrix(matrix, isprecise=False):
    """Return quaternion from rotation matrix.

    If isprecise is True, the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.

    >>> q = quaternion_from_matrix(numpy.identity(4), True)
    >>> numpy.allclose(q, [1, 0, 0, 0])
    True
    >>> q = quaternion_from_matrix(numpy.diag([1, -1, -1, 1]))
    >>> numpy.allclose(q, [0, 1, 0, 0]) or numpy.allclose(q, [0, -1, 0, 0])
    True
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R, True)
    >>> numpy.allclose(q, [0.9981095, 0.0164262, 0.0328524, 0.0492786])
    True
    >>> R = [[-0.545, 0.797, 0.260, 0], [0.733, 0.603, -0.313, 0],
    ...      [-0.407, 0.021, -0.913, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.19069, 0.43736, 0.87485, -0.083611])
    True
    >>> R = [[0.395, 0.362, 0.843, 0], [-0.626, 0.796, -0.056, 0],
    ...      [-0.677, -0.498, 0.529, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.82336615, -0.13610694, 0.46344705, -0.29792603])
    True
    >>> R = random_rotation_matrix()
    >>> q = quaternion_from_matrix(R)
    >>> is_same_transform(R, quaternion_matrix(q))
    True
    >>> is_same_quaternion(quaternion_from_matrix(R, isprecise=False),
    ...                    quaternion_from_matrix(R, isprecise=True))
    True
    >>> R = euler_matrix(0.0, 0.0, numpy.pi/2.0)
    >>> is_same_quaternion(quaternion_from_matrix(R, isprecise=False),
    ...                    quaternion_from_matrix(R, isprecise=True))
    True

    """
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q = np.empty((4, ))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]

    return [q[1],q[2],q[3],q[0]]


if __name__ == '__main__':
    main()

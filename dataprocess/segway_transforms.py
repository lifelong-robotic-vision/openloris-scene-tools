#!/usr/bin/env python2
import subprocess
import tf
import numpy as np

"""
Original calibration results
Format: [(source, parent_frame, child_frame, [x,y,z,qx,qy,qz,qw])]
The name 'parent_frame' and 'child_frame' follows ROS convention
i.e. the transform is from child_frame coordinates to parent_frame coordinates, or T_parent_child
i.e. it is equivalent to child pose in parent_frame
"""
original_tfs = [
    # robot_cal_tools
    ('rct', 'marker', 'd400_color', [-0.0318876, 0.0490709, -0.00575465, -0.7078259, -0.0133337, -0.0050605, 0.7062429]),
    # autoware -- inaccurate
    ('autoware', 'laser', 'd400_color', [0.081789500607921867, 0.023429544603214288, 0.076229065590804543, 0.5339199, -0.5124351, 0.467916, -0.4831092]),
    # LaserCamCal
    ('lcc', 'laser', 'd400_color_848', [0.08478335446665922, 0.039202142419932993, -0.051659305275160064, -0.505557706807341, 0.49356442986834465, -0.49742698135424057, 0.5033606628226388]),
    ('lcc', 'laser', 'd400_color', [0.08600746628046706, 0.04385715594152477, -0.07966534028335034, -0.49122917294737645, 0.48696266322651327, -0.5084900351834007, 0.5128344258997841]),
    ('lcc', 'laser', 't265_fisheye1', [0.10330055463714409, 0.053223074731757185, -0.09357657662078833, -0.4921622981856804, 0.4930522577087068, -0.4992976698383884, 0.5151481149212255]),
    ('lcc', 'laser', 't265_fisheye2', [0.10274418582817242, -0.013594041093129446, -0.08823912424751025, -0.4909585515209887, 0.49519929691744397, -0.5000838101963191, 0.513472043835314]),
    # basalt -- IMU calibration is inaccurate
    ('basalt', 't265_imu', 't265_fisheye1', [-0.001057399052113554, 0.0020920342503451528, -0.0002740477123929344, -0.6529740355381054, 0.005953328667198988, -0.05821788685357764, 0.7551159807878293]),
    ('basalt', 't265_imu', 'marker', [0.032653488512015247, -0.07086889526625034, 0.02004296168173806, 0.08673214386231039, -0.022916885982446849, -0.04000986672047097, 0.9951640880392164]),
    # kalibr
    ('kalibr', 't265_fisheye1', 'd400_color', [0.01195077, -0.02124539, -0.01098818, -0.00231707, 0.00076812, 0.00001986, 0.99999702]),
    ('kalibr', 't265_fisheye2', 't265_fisheye1', [-0.06332501, -0.00012825, -0.00072183, 0.00201712, 0.00310801, 0.00168845, 0.99999171]),
    # Segway odom calibration -- z-axis translations are manually estimated
    ('segway', 'base_link', 'd400_color', [0.2264836849091656, -0.05114194035652147, 0.916, -0.49676229968284147, 0.49987958871297716, -0.4951068126935409, 0.508150428934585]),
    ('segway', 'base_link', 't265_fisheye1', [0.2345087047380146, 0.03755418436489752, 0.896, -0.49608636652776034, 0.5056954958519232, -0.49632547324982473, 0.5018280652013194]),
    # D435i factory calibration
    ('realsense', 'd400_color', 'd400_depth', [0.014881294220686, -2.32995425903937e-05, 0.000588475959375501, 0.00174536, 0.00293073, -0.00191947, 0.99999237]),
    ('realsense', 'd400_color', 'd400_imu', [0.0203127935528755, -0.0051032523624599, -0.0112013882026076, 0.00174536, 0.00293073, -0.00191947, 0.99999237]),
    # T265 factory calibration
    ('realsense', 't265_fisheye1', 't265_fisheye2', [0.0639765113592148, 0.000148267135955393, -0.000398468371713534, 0.00188497, 0.00347595, 0.00154952, 0.999991]),
    ('realsense', 't265_fisheye1', 't265_imu', [0.0106999985873699, 7.27595761418343e-12, -2.91038304567337e-11, 0.00554841, 0.00136098, 0.99998062, -0.00245956]),
    ('realsense', 't265_fisheye1', 't265_pose', [-0.031988475471735, -8.31645156722516e-05, 0.000155729198013432, 0.9999806248123011, 0.0024595551543427716, -0.005548407501436771, -0.0013609838693179235]),
    #('realsense', 't265_imu', 't265_fisheye1', [0.0106992106884718, -5.27950360265095e-05, -0.000118661890155636, 0.00554841, 0.00136098, 0.99998062, 0.00245956]),
]

intrinsics = {
    'd400_color': {
        'distortion_model': 'plumb_bob',
        'D': [0.0, 0.0, 0.0, 0.0, 0.0],
        'K': [611.4509887695312, 0.0, 433.2039794921875, 0.0, 611.4857177734375, 249.4730224609375, 0.0, 0.0, 1.0],
        'R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'P': [611.4509887695312, 0.0, 433.2039794921875, 0.0, 0.0, 611.4857177734375, 249.4730224609375, 0.0, 0.0, 0.0, 1.0, 0.0],
    },
    'd400_depth': {
        'distortion_model': "plumb_bob",
        'D': [0.0, 0.0, 0.0, 0.0, 0.0],
        'K': [422.1437072753906, 0.0, 427.0083312988281, 0.0, 422.1437072753906, 245.2209014892578, 0.0, 0.0, 1.0],
        'R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'P': [422.1437072753906, 0.0, 427.0083312988281, 0.0, 0.0, 422.1437072753906, 245.2209014892578, 0.0, 0.0, 0.0, 1.0, 0.0],
    },
    't265_fisheye1': {

    }
}

def get_tfs():
    global original_tfs
    return tfs_to_tree(original_tfs)

def tfs_to_tree(original_tfs):
    tfs = TFParser(original_tfs)
    t_base_color = tfs.get_tf_matrix('segway', 'base_link', 'd400_color')
    t_color_fisheye = tfs.get_tf_matrix_inverse('kalibr', 't265_fisheye1', 'd400_color')
    t_base_fisheye = np.dot(t_base_color, t_color_fisheye)
    t_color_mocap = tfs.get_tf_matrix_inverse('rct', 'marker', 'd400_color')
    t_base_mocap = np.dot(t_base_color, t_color_mocap)
    t_color_laser = tfs.get_tf_matrix_inverse('lcc', 'laser', 'd400_color')
    t_base_laser = np.dot(t_base_color, t_color_laser)
    tf_tree = [
        ('tree', 'base_link', 'd400_color', tf_matrix_to_values(t_base_color)),
        ('tree', 'base_link', 't265_fisheye1', tf_matrix_to_values(t_base_fisheye)),
        ('tree', 'base_link', 'marker', tf_matrix_to_values(t_base_mocap)),
        ('tree', 'base_link', 'laser', tf_matrix_to_values(t_base_laser)),
        ('tree', 'd400_color', 'd400_depth', tfs.get_tf_values('realsense', 'd400_color', 'd400_depth')),
        ('tree', 'd400_color', 'd400_imu', tfs.get_tf_values('realsense', 'd400_color', 'd400_imu')),
        ('tree', 't265_fisheye1', 't265_fisheye2', tfs.get_tf_values('realsense', 't265_fisheye1', 't265_fisheye2')),
        ('tree', 't265_fisheye1', 't265_imu', tfs.get_tf_values('realsense', 't265_fisheye1', 't265_imu')),
        #('tree', 't265_fisheye1', 't265_pose', tfs.get_tf_values('realsense', 't265_fisheye1', 't265_pose')),
    ]
    return tf_tree

def tf_matrix_to_values(matrix):
    t = list(matrix[:3, 3])
    q = list(tf.transformations.quaternion_from_matrix(matrix))
    return t + q

def tf_values_to_matrix(values):
    angles = tf.transformations.euler_from_quaternion(values[3:])
    return tf.transformations.compose_matrix(angles=angles, translate=values[:3])

"""
Get the best transform matrix
"""
def get_tf_matrix(parent, child):
    if parent == child:
        return tf_values_to_matrix([0,0,0,0,0,0,1])
    global original_tfs
    tfs = TFParser(tfs_to_tree(original_tfs))
    t = tfs.get_tf_matrix('tree', parent, child)
    if t is not None: return t
    t = tfs.get_tf_matrix_inverse('tree', child, parent)
    if t is not None: return t
    t1 = tfs.get_tf_matrix_inverse('tree', 'base_link', parent)
    t2 = tfs.get_tf_matrix('tree', 'base_link', child)
    if t1 is not None and t2 is not None:
        return np.dot(t1, t2)
    if child.startswith('t265'):
        t1 = get_tf_matrix(parent, 't265_fisheye1')
        t2 = get_tf_matrix('t265_fisheye1', child)
        if t1 is not None and t2 is not None:
            return np.dot(t1, t2)
    if parent.startswith('t265'):
        t1 = get_tf_matrix('t265_fisheye1', parent)
        t2 = get_tf_matrix(child, 't265_fisheye1')
        if t1 is not None and t2 is not None:
            return tf.transformations.inverse_matrix(np.dot(t2, t1))
    if child.startswith('d400'):
        t1 = get_tf_matrix(parent, 'd400_color')
        t2 = get_tf_matrix('d400_color', child)
        if t1 is not None and t2 is not None:
            return np.dot(t1, t2)
    if parent.startswith('d400'):
        t1 = get_tf_matrix('d400_color', parent)
        t2 = get_tf_matrix(child, 'd400_color')
        if t1 is not None and t2 is not None:
            return tf.transformations.inverse_matrix(np.dot(t2, t1))
    exit('Not sure how to get tf of %s from %s' % (parent, child))

def parse_opencv_matrix():
    m_str = """
9.3876782240032049e-03, 2.7313453163310519e-02,
       9.9958283637418410e-01, 1.0274418582817242e-01,
       -9.9980277127851036e-01, 1.7751766935890406e-02,
       8.9046793577878623e-03, -1.3594041093129446e-02,
       -1.7501144001857809e-02, -9.9946928419384162e-01,
       2.7474714041764919e-02, -8.8239124247510251e-02, 0., 0., 0., 1. """
    m = m_str.strip(' []').replace('\n', ' ').split(',')
    m = [float(v.strip()) for v in m]
    m = [m[0:4], m[4:8], m[8:12], m[12:]]
    print (m)
    t = [r[3] for r in m[:3]]
    q = tf.transformations.quaternion_from_matrix(m)
    print ('matrix => translation, quaternion:')
    print (t + list(q))

#parse_opencv_matrix()

def rotation_matrix_to_quaternion():
    r_str = """
   0.999926         0.00490391      -0.0111033
   0.00493412      -0.999984         0.00269462
  -0.0110899       -0.00274921      -0.999935
    """
    r = r_str.strip().split('\n')
    r = [row.strip().split() for row in r]
    r = [[float(v) for v in row] for row in r]
    r[0].append(0)
    r[1].append(0)
    r[2].append(0)
    r.append([0,0,0,1])
    print (r)
    q = tf.transformations.quaternion_from_matrix(r)
    print ('rotation => quaternion:')
    print (', '.join(map(str, q)))

#rotation_matrix_to_quaternion()

class TFParser(object):
    __tfs = {}
    def __init__(self, tfs):
        for tf in tfs:
            if len(tf) is not 4 or len(tf[3]) is not 7:
                print (tf)
                exit('Invalid tf format')
            self.__tfs[(tf[0], tf[1], tf[2])] = tf[3]

    def get_tf_values(self, source, parent, child):
        if (source, parent, child) in self.__tfs:
            return self.__tfs[(source, parent, child)]
        return None

    def get_tf_matrix(self, source, parent, child):
        if (source, parent, child) in self.__tfs:
            t = self.__tfs[(source, parent, child)]
            angles = tf.transformations.euler_from_quaternion(t[3:])
            return tf.transformations.compose_matrix(angles=angles, translate=t[:3])
        return None

    def get_tf_matrix_inverse(self, source, parent, child):
        m = self.get_tf_matrix(source, parent, child)
        return None if m is None else tf.transformations.inverse_matrix(m)

def publish_tf(frame1, frame2, tf):
    subprocess.Popen(['static_transform_publisher', \
        str(tf[0]), str(tf[1]), str(tf[2]), str(tf[3]), str(tf[4]), str(tf[5]), str(tf[6]), \
        frame1, frame2])

def compare_tfs(tf1, tf2):
    t1 = tf1[:3, 3:]
    t2 = tf2[:3, 3:]
    dist = np.linalg.norm(t1 - t2)
    angle,_,_ = tf.transformations.rotation_from_matrix(np.dot(np.linalg.inv(tf1), tf2))
    print ('translation diff %f (m), rotation diff %f (deg)' % (dist, angle / np.pi * 180.))

def verify_calibration():
    """
    print estimated transforms from D435i to T265 from different calibration sources
    """
    np.set_printoptions(suppress=True)
    global original_tfs
    tfs = TFParser(original_tfs)

    t_fisheye_color_c = tfs.get_tf_matrix('kalibr', 't265_fisheye1', 'd400_color')
    print ('------------------------------------')
    print ('T_fisheye_color from kalibr:')
    print (t_fisheye_color_c)
    compare_tfs(t_fisheye_color_c, t_fisheye_color_c)

    t_fisheye_color_m = np.dot(np.dot(\
        tfs.get_tf_matrix_inverse('basalt', 't265_imu', 't265_fisheye1'), \
        tfs.get_tf_matrix('basalt', 't265_imu', 'marker')), \
        tfs.get_tf_matrix('rct', 'marker', 'd400_color'))
    print ('------------------------------------')
    print ('T_fisheye_color from basalt+rct:')
    print (t_fisheye_color_m)
    compare_tfs(t_fisheye_color_m, t_fisheye_color_c)

    t_fisheye_color_o = np.dot(tfs.get_tf_matrix_inverse('segway', 'base_link', 't265_fisheye1'), \
        tfs.get_tf_matrix('segway', 'base_link', 'd400_color'))
    print ('------------------------------------')
    print ('T_fisheye_color from segway:')
    print (t_fisheye_color_o)
    compare_tfs(t_fisheye_color_o, t_fisheye_color_c)

    t_fisheye_color_l = np.dot(tfs.get_tf_matrix_inverse('lcc', 'laser', 't265_fisheye1'), \
        tfs.get_tf_matrix('lcc', 'laser', 'd400_color'))
    print ('------------------------------------')
    print ('T_fisheye_color from LaserCamCal:')
    print (t_fisheye_color_l)
    compare_tfs(t_fisheye_color_l, t_fisheye_color_c)


    t_fisheyes_r = tfs.get_tf_matrix('realsense', 't265_fisheye1', 't265_fisheye2')
    t_fisheyes_l = np.dot(tfs.get_tf_matrix_inverse('lcc', 'laser', 't265_fisheye1'), \
        tfs.get_tf_matrix('lcc', 'laser', 't265_fisheye2'))
    print ('------------------------------------')
    print (t_fisheyes_r)
    print ('T_fisheye1_2 from LaserCamCal:')
    print (t_fisheyes_l)
    compare_tfs(t_fisheyes_l, t_fisheyes_r)


    t_l1 = tfs.get_tf_matrix('lcc', 'laser', 'd400_color')
    t_l2 = tfs.get_tf_matrix('lcc', 'laser', 'd400_color_848')
    print ('------------------------------------')
    print (t_l1)
    print ('T_laser_color from LaserCamCal:')
    print (t_l2)
    compare_tfs(t_l1, t_l2)



    tfs = tfs_to_tree(original_tfs)
    for t in tfs:
        #publish_tf(t[1], t[2], t[3])
        print (t[1], t[2], t[3])

def print_tf():
    targets = ['d400_color', 'd400_depth', 'd400_imu', 't265_fisheye1', 't265_fisheye2', 't265_imu']
    for target in targets:
        m = get_tf_matrix('base_link', target)
        v = tf_matrix_to_values(m)
        print("('%s', '%s', %s)," % ('base_link', target, str(v)))

if __name__ == '__main__':
    verify_calibration()

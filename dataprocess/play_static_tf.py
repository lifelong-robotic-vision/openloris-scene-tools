#!/usr/bin/env python2
import subprocess
import tf
import sys
import rosbag
import rospy
import time
from tf2_msgs.msg import TFMessage

def reset_stamp(msg):
    t = rospy.Time.now()
    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
        msg.header.stamp = t
    elif hasattr(msg, 'transforms'):
        for id in xrange(len(msg.transforms)):
            msg.transforms[id].header.stamp = t
    return msg

def publish_loop(pub, msg, interval=0.01):
    while True:
        pub.publish(reset_stamp(msg))
        time.sleep(interval)

def main():
    bagfile = sys.argv[1]
    static_tf_msg = None
    with rosbag.Bag(bagfile) as bag:
        for topic,msg,t in bag.read_messages('/tf_static'):
            if static_tf_msg is None:
                static_tf_msg = msg
            else:
                for transform in msg.transforms:
                    static_tf_msg.transforms.append(transform)

    if static_tf_msg is None:
        print ('No message on /tf_static')
        exit(0)

    print ('Will loop %d static transforms on /tf' % len(msg.transforms))

    rospy.init_node("static_tf_looper")
    pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
    publish_loop(pub, static_tf_msg)


def publish_tf(frame1, frame2, tf):
    subprocess.Popen(['static_transform_publisher', \
        str(tf[0]), str(tf[1]), str(tf[2]), str(tf[3]), str(tf[4]), str(tf[5]), str(tf[6]), \
        frame1, frame2])

def compare_tfs(tf1, tf2):
    t1 = tf1[:3, 3:]
    t2 = tf2[:3, 3:]
    dist = np.linalg.norm(t1 - t2)
    angle,_,_ = tf.transformations.rotation_from_matrix(np.dot(np.linalg.inv(tf1), tf2))
    print 'translation diff %f (m), rotation diff %f (deg)' % (dist, angle / np.pi * 180.)

def verify_calibration():
    """
    print estimated transforms from D435i to T265 from different calibration sources
    """
    np.set_printoptions(suppress=True)
    global original_tfs
    tfs = TFParser(original_tfs)

    t_fisheye_color_c = tfs.get_tf_matrix('kalibr', 't265_fisheye1', 'd400_color')
    print '------------------------------------'
    print 'T_fisheye_color from kalibr:'
    print t_fisheye_color_c
    compare_tfs(t_fisheye_color_c, t_fisheye_color_c)

    t_fisheye_color_m = np.dot(np.dot(\
        tfs.get_tf_matrix_inverse('basalt', 't265_imu', 't265_fisheye1'), \
        tfs.get_tf_matrix('basalt', 't265_imu', 'marker')), \
        tfs.get_tf_matrix('rct', 'marker', 'd400_color'))
    print '------------------------------------'
    print 'T_fisheye_color from basalt+rct:'
    print t_fisheye_color_m
    compare_tfs(t_fisheye_color_m, t_fisheye_color_c)

    t_fisheye_color_o = np.dot(tfs.get_tf_matrix_inverse('segway', 'base_link', 't265_fisheye1'), \
        tfs.get_tf_matrix('segway', 'base_link', 'd400_color'))
    print '------------------------------------'
    print 'T_fisheye_color from segway:'
    print t_fisheye_color_o
    compare_tfs(t_fisheye_color_o, t_fisheye_color_c)

    t_fisheye_color_l = np.dot(tfs.get_tf_matrix_inverse('lcc', 'laser', 't265_fisheye1'), \
        tfs.get_tf_matrix('lcc', 'laser', 'd400_color'))
    print '------------------------------------'
    print 'T_fisheye_color from LaserCamCal:'
    print t_fisheye_color_l
    compare_tfs(t_fisheye_color_l, t_fisheye_color_c)


    t_fisheyes_r = tfs.get_tf_matrix('realsense', 't265_fisheye1', 't265_fisheye2')
    t_fisheyes_l = np.dot(tfs.get_tf_matrix_inverse('lcc', 'laser', 't265_fisheye1'), \
        tfs.get_tf_matrix('lcc', 'laser', 't265_fisheye2'))
    print '------------------------------------'
    print t_fisheyes_r
    print 'T_fisheye1_2 from LaserCamCal:'
    print t_fisheyes_l
    compare_tfs(t_fisheyes_l, t_fisheyes_r)


    t_l1 = tfs.get_tf_matrix('lcc', 'laser', 'd400_color')
    t_l2 = tfs.get_tf_matrix('lcc', 'laser', 'd400_color_848')
    print '------------------------------------'
    print t_l1
    print 'T_laser_color from LaserCamCal:'
    print t_l2
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
    main()

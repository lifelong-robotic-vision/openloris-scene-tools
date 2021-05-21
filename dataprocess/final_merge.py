#!/usr/bin/env python

# Copyright (C) <2019-2021> Intel Corporation
# SPDX-License-Identifier: MIT
# Author: Xuesong Shi

"""
1. truncate bag by filtering out messages out of a given timestamp range
2. add a bias to all unix stamps in the bag
3. remove pose stream from T265
"""
import rospy
import rosbag
import sys
import os
from rospy.rostime import Time, Duration
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from realsense2_camera.msg import IMUInfo
import tf
import numpy as np
import cv2
import cv_bridge

#gt_source = 'mocap'
gt_source = 'hector'
gt_source = 'gaussian'
tf_source = 'segway'
tf_source = 'gaussian'
if tf_source == 'segway': import segway_transforms as transform_data
elif tf_source == 'gaussian': import gaussian_transforms as transform_data

def main():
    infiles = ['synced.bag', 'record-d400.bag', 'record-t265.bag'] if len(sys.argv) == 1 else sys.argv[1:]
    outfile = 'final.bag'
    outbag = rosbag.Bag(outfile, 'w')
    write_exposure_to_seq = True
    write_aligned_depth = True

    if write_aligned_depth:
        aligned_depth_folder = 'aligned_depth'
        if not os.path.exists(aligned_depth_folder):
            print('Did not find folder of aligned depth images')
            aligned_depth_filenames = []
        else:
            aligned_depth_filenames = sorted([s for s in os.listdir(aligned_depth_folder) if s.endswith('.png')])
            print('Found %d images in %s' % (len(aligned_depth_filenames), aligned_depth_folder))

    with rosbag.Bag(infiles[0]) as refbag:
        tstart = Time.from_sec(refbag.get_start_time())
    add_static_tf(outbag, tstart)

    # { in_topic: (out_topic, frame_id) }
    image_topics = {
        '/device_0/sensor_1/Color_0/image/data': ('/d400/color/image_raw', 'd400_color'),
        '/device_0/sensor_0/Depth_0/image/data': ('/d400/depth/image_raw', 'd400_depth'),
        '/device_0/sensor_0/Fisheye_1/image/data': ('/t265/fisheye1/image_raw', 't265_fisheye1'),
        '/device_0/sensor_0/Fisheye_2/image/data': ('/t265/fisheye2/image_raw', 't265_fisheye2'),
    }
    imu_topics = {
        '/device_0/sensor_2/Accel_0/imu/data': ('/d400/accel/sample', 'd400_imu'),
        '/device_0/sensor_2/Gyro_0/imu/data': ('/d400/gyro/sample', 'd400_imu'),
        '/device_0/sensor_0/Accel_0/imu/data': ('/t265/accel/sample', 't265_imu'),
        '/device_0/sensor_0/Gyro_0/imu/data': ('/t265/gyro/sample', 't265_imu'),
    }
    camera_info_topics = {
        '/device_0/sensor_1/Color_0/info/camera_info': ('/d400/color/camera_info', 'd400_color'),
        '/device_0/sensor_0/Depth_0/info/camera_info': ('/d400/depth/camera_info', 'd400_depth'),
        '/device_0/sensor_0/Fisheye_1/info/camera_info': ('/t265/fisheye1/camera_info', 't265_fisheye1'),
        '/device_0/sensor_0/Fisheye_2/info/camera_info': ('/t265/fisheye2/camera_info', 't265_fisheye2'),
    }
    imu_info_topics = {
        '/device_0/sensor_2/Accel_0/imu_intrinsic': ('/d400/accel/imu_info', 'imu_accel'),
        '/device_0/sensor_2/Gyro_0/imu_intrinsic': ('/d400/gyro/imu_info', 'imu_gyro'),
        '/device_0/sensor_0/Accel_0/imu_intrinsic': ('/t265/accel/imu_info', 'imu_accel'),
        '/device_0/sensor_0/Gyro_0/imu_intrinsic': ('/t265/gyro/imu_info', 'imu_gyro'),
    }

    def write_image(outbag, topic, msg):
        if write_aligned_depth and topic == '/d400/depth/image_raw':
            filename = '%f.png' % msg.header.stamp.to_sec()
            if filename not in aligned_depth_filenames:
                dist = [abs(float(f.rstrip('.png')) - msg.header.stamp.to_sec()) for f in aligned_depth_filenames]
                stamp_diff_tol = 0.005 # 5ms
                if min(dist) < stamp_diff_tol:
                    filename = aligned_depth_filenames[dist.index(min(dist))]
                else:
                    print('Could not find aligned depth image at %.9f. Closest is %s' % (msg.header.stamp.to_sec(), aligned_depth_filenames[dist.index(min(dist))]))
                    return
            im = cv2.imread(aligned_depth_folder + '/' + filename, cv2.IMREAD_UNCHANGED)
            msg_aligned = cv_bridge.CvBridge().cv2_to_imgmsg(im, encoding='mono16')
            msg_aligned.header = msg.header
            msg_aligned.header.frame_id = 'd400_color'
            outbag.write('/d400/aligned_depth_to_color/image_raw', msg_aligned, msg.header.stamp)
        outbag.write(topic, msg, msg.header.stamp)

    for infile in infiles:
        with rosbag.Bag(infile) as inbag:
            print ("-------------------- Input: %s ------------------------------" % infile)
            print (inbag)
            image_buffers = {}
            exposure_buffers = {}
            for topic,msg,t in inbag.read_messages():
                if topic == '/odom' or topic == '/scan' or topic == '/rslidar_packets':
                    outbag.write(topic, msg, msg.header.stamp)
                elif topic == '/rslidar_packets_difop':
                    outbag.write(topic, msg, msg.stamp)
                elif gt_source == 'hector' and topic == '/slam_out_pose':
                    base_pose = transform_pose_msg(msg, 'base_link', 'base_link')
                    base_pose.header.frame_id = 'gt_map'
                    #outbag.write('/gt_base', base_pose, msg.header.stamp)
                    #outbag.write('/gt_d400_color', transform_pose_msg(base_pose, 'base_link', 'd400_color'), msg.header.stamp)
                    #outbag.write('/gt_t265_fisheye1', transform_pose_msg(base_pose, 'base_link', 't265_fisheye1'), msg.header.stamp)
                    outbag.write('/gt', pose_to_tf(base_pose, 'base_link'), msg.header.stamp)
                elif gt_source == 'gaussian' and topic == '/v5_current_pose':
                    base_pose = PoseStamped()
                    base_pose.header = msg.header
                    base_pose.pose = msg.pose.pose
                    base_pose.header.frame_id = 'gt_map'
                    #outbag.write('/gt_base', base_pose, msg.header.stamp)
                    #outbag.write('/gt_d400_color', transform_pose_msg(base_pose, 'base_link', 'd400_color'), msg.header.stamp)
                    #outbag.write('/gt_t265_fisheye1', transform_pose_msg(base_pose, 'base_link', 't265_fisheye1'), msg.header.stamp)
                    outbag.write('/gt', pose_to_tf(base_pose, 'base_link'), msg.header.stamp)
                elif gt_source == 'mocap' and topic == '/vrpn_client_node/RigidBody1/pose':
                    base_pose = transform_pose_msg(msg, 'marker', 'base_link')
                    base_pose.header.frame_id = 'gt_map'
                    #outbag.write('/gt_base', base_pose, msg.header.stamp)
                    #outbag.write('/gt_d400_color', transform_pose_msg(base_pose, 'base_link', 'd400_color'), msg.header.stamp)
                    #outbag.write('/gt_t265_fisheye1', transform_pose_msg(base_pose, 'base_link', 't265_fisheye1'), msg.header.stamp)
                    outbag.write('/gt', pose_to_tf(base_pose, 'base_link'), msg.header.stamp)
                elif topic in image_topics:
                    msg.header.frame_id = image_topics[topic][1]
                    if write_exposure_to_seq:
                        if topic in exposure_buffers and t in exposure_buffers[topic]:
                            msg.header.seq = exposure_buffers[topic].pop(t)
                            write_image(outbag, image_topics[topic][0], msg)
                        else:
                            image_buffers.setdefault(topic, {})[t] = msg
                    else:
                        write_image(outbag, image_topics[topic][0], msg)
                elif write_exposure_to_seq and topic.rstrip('metadata') + 'data' in image_topics and msg.key == 'Actual Exposure':
                    image_topic = topic.rstrip('metadata') + 'data'
                    if image_topic in image_buffers and t in image_buffers[image_topic]:
                        image = image_buffers[image_topic].pop(t)
                        image.header.seq = int(msg.value)
                        write_image(outbag, image_topics[image_topic][0], image)
                    else:
                        exposure_buffers.setdefault(image_topic, {})[t] = int (msg.value)
                elif topic in camera_info_topics:
                    msg.header.stamp = tstart
                    msg.header.frame_id = camera_info_topics[topic][1]
                    msg.P = list(msg.P)
                    msg.P[0] = msg.K[0]
                    msg.P[2] = msg.K[2]
                    msg.P[5] = msg.K[4]
                    msg.P[6] = msg.K[5]
                    msg.P[10] = 1.
                    msg.R = list(msg.R)
                    msg.R[0] = 1.
                    msg.R[4] = 1.
                    msg.R[8] = 1.
                    outbag.write(camera_info_topics[topic][0], msg, tstart)
                    if write_aligned_depth and camera_info_topics[topic][0] == '/d400/color/camera_info':
                        outbag.write('/d400/aligned_depth_to_color/camera_info', msg, tstart)
                elif topic in imu_info_topics:
                    msg_info = IMUInfo()
                    if hasattr(msg_info, 'header'):
                        msg_info.header.stamp = tstart
                        msg_info.header.frame_id = imu_info_topics[topic][1]
                    else:
                        msg_info.frame_id = imu_info_topics[topic][1]
                    msg_info.data = msg.data
                    msg_info.noise_variances = msg.noise_variances
                    msg_info.bias_variances = msg.bias_variances
                    outbag.write(imu_info_topics[topic][0], msg_info, tstart)
                elif topic in imu_topics:
                    msg.header.frame_id = imu_topics[topic][1]
                    outbag.write(imu_topics[topic][0], msg, msg.header.stamp)

            for topic in image_buffers:
                if len(image_buffers[topic]) != 0:
                    print ('Error: discarded %d images without exposure data on %s. This should not happen!' % len(image_buffers[topic]), topic)

    outbag.close()
    outbag = rosbag.Bag(outfile)
    print ("------------------- Output: %s ------------------------------" % outfile)
    print (outbag)

def add_static_tf(outbag, t):
    msg = TFMessage()
    def fill_transform(parent, child, tf_values):
        ts = TransformStamped()
        ts.header.frame_id = parent
        ts.child_frame_id = child
        ts.transform.translation.x = tf_values[0]
        ts.transform.translation.y = tf_values[1]
        ts.transform.translation.z = tf_values[2]
        ts.transform.rotation.x = tf_values[3]
        ts.transform.rotation.y = tf_values[4]
        ts.transform.rotation.z = tf_values[5]
        ts.transform.rotation.w = tf_values[6]
        msg.transforms.append(ts)

    tfs = transform_data.get_tfs()
    for tt in tfs:
        fill_transform(tt[1], tt[2], tt[3])
    outbag.write('/tf_static', msg, t)

def transform_pose_msg(msg, child_frame_current, child_frame_new):
    """
    transform pose in given msg
    """
    def pose_msg_to_matrix(msg):
        translate = [msg.position.x, msg.position.y, msg.position.z]
        angles = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        return tf.transformations.compose_matrix(angles=angles, translate=translate)
    def matrix_to_pose_msg(m):
        msg = Pose()
        msg.position.x = m[0,3]
        msg.position.y = m[1,3]
        msg.position.z = m[2,3]
        q = tf.transformations.quaternion_from_matrix(m)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        return msg
    t_current_new = transform_data.get_tf_matrix(child_frame_current, child_frame_new)
    t_world_current = pose_msg_to_matrix(msg.pose)
    t_world_new = np.dot(t_world_current, t_current_new)
    ret = PoseStamped()
    ret.header = msg.header
    ret.pose = matrix_to_pose_msg(t_world_new)
    return ret

def pose_to_tf(pose_msg, child_frame_id):
    ts = TransformStamped()
    ts.header = pose_msg.header
    ts.child_frame_id = child_frame_id
    ts.transform.translation = pose_msg.pose.position
    ts.transform.rotation = pose_msg.pose.orientation
    msg = TFMessage()
    msg.transforms.append(ts)
    return msg

if __name__ == '__main__':
    main()
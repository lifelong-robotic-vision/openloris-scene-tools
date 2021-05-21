#!/usr/bin/env python2

# Copyright (C) <2019> Intel Corporation
# SPDX-License-Identifier: MIT
# Author: Xuesong Shi

import rosbag
import numpy as np
import sys
import tf
from geometry_msgs.msg import PoseStamped, Pose

def main():
    rot = np.matrix([[-0.005550530371,  0.999984447024, -0.000545272573],
                        [-0.99997132828,  -0.005553264765, -0.005148190746],
                        [-0.005151138719,  0.00051668175,   0.999986599315]])

    trans = np.matrix([[-14.084838405548],
                          [ 13.831934501961],
                          [ -0.097715618173]])

    T = np.zeros((4,4))
    T[:3, :3] = rot
    T[:3, [3]] = trans
    T[3, 3] = 1
    print T

    with rosbag.Bag('hector-pose.bag') as inbag:
        outfile = 'hector-pose-trans.bag'
        outbag = rosbag.Bag(outfile, 'w')
        for topic,msg,t in inbag.read_messages(['/slam_out_pose']):
            outbag.write(topic, transform_pose(msg, T), msg.header.stamp)
        inbag.close()
        outbag.close()

def transform_pose(msg, t_new_world):
    """
    transform pose to different world frame
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
    # there is a map coordinate shift for the corridor data
    #msg.pose.position.x += 16.384
    #msg.pose.position.y += 16.384
    t_world_target = pose_msg_to_matrix(msg.pose)
    t_new_target = np.dot(t_new_world, t_world_target)
    ret = PoseStamped()
    ret.header = msg.header
    ret.pose = matrix_to_pose_msg(t_new_target)
    return ret

if __name__=="__main__":
    main()

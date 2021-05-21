#!/usr/bin/env python2

# Copyright (C) <2019> Intel Corporation
# SPDX-License-Identifier: MIT
# Author: Xuesong Shi

"""
1. sync between topics
2. add an offset to all time stamps
"""
import rosbag
import sys
import yaml
import os
from geometry_msgs.msg import PoseStamped
from rospy.rostime import Time, Duration

def main():
    topic_in = '/poseupdate'
    topic_out = '/slam_out_pose'

    files = sys.argv[1:]

    for file in files:
        with rosbag.Bag(file) as inbag:
            outfile = file.rstrip('.bag') + '-converted.bag'
            outbag = rosbag.Bag(outfile, 'w')
            for topic,msg,t in inbag.read_messages():
                if topic == topic_in:
                    pose = PoseStamped()
                    pose.header = msg.header
                    pose.pose = msg.pose.pose
                    outbag.write(topic_out, pose, t)
                else:
                    outbag.write(topic, msg, t)

            outbag.close()
            outbag = rosbag.Bag(outfile)
            print ("-------------------- Input: %s ------------------------------" % file)
            print (inbag)
            print ("------------------- Output: %s ------------------------------" % outfile)
            print (outbag)


if __name__ == '__main__':
    main()

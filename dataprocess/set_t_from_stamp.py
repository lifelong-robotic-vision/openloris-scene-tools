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
    files = sys.argv[1:]

    for file in files:
        with rosbag.Bag(file) as inbag:
            outfile = file.rstrip('.bag') + '-conv-t.bag'
            outbag = rosbag.Bag(outfile, 'w')
            for topic,msg,t in inbag.read_messages():
                if hasattr(msg, 'header'):
                    outbag.write(topic, msg, msg.header.stamp)
                elif hasattr(msg, 'transforms') and len(msg.transforms) == 1:
                    outbag.write(topic, msg, msg.transforms[0].header.stamp)
                else:
                    print msg
                    exit('Unsupported message type!')

            outbag.close()
            outbag = rosbag.Bag(outfile)
            print ("-------------------- Input: %s ------------------------------" % file)
            print (inbag)
            print ("------------------- Output: %s ------------------------------" % outfile)
            print (outbag)


if __name__ == '__main__':
    main()

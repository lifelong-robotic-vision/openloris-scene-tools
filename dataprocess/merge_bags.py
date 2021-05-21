#!/usr/bin/env python2

# Copyright (C) <2019> Intel Corporation
# SPDX-License-Identifier: MIT
# Author: Xuesong Shi

"""
merge given bag files into a single one
"""
import rosbag
import sys

def main():
    files = sys.argv[1:]
    outfile = 'merged.bag'
    outbag = rosbag.Bag(outfile, 'w')

    for file in files:
        with rosbag.Bag(file) as inbag:
            print ("-------------------- Input: %s ------------------------------" % file)
            print (inbag)
            for topic,msg,t in inbag.read_messages():
                outbag.write(topic, msg, t)

    outbag.close()
    outbag = rosbag.Bag(outfile)
    print ("------------------- Output: %s ------------------------------" % outfile)
    print (outbag)

if __name__ == '__main__':
    main()

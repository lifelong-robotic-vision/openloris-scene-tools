#!/usr/bin/env python2

# Copyright (C) <2019> Intel Corporation
# SPDX-License-Identifier: MIT
# Author: Xuesong Shi

"""
Remove duplicated messages in one or more ROS bags.
"""
import rosbag
import sys
import os
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--topic', type=str, default='',\
        help='topic(s) having duplicates, use comma to separate topics, leave blank to check all')
    parser.add_argument('-o', '--outfolder', type=str, default='removed_duplicates',\
        help='path to save the processed bags, which will have the same filenames as the input bags')

    args, infiles = parser.parse_known_args()
    if len(infiles) == 0:
        exit('Usage: %s [-s topic1[,topic2,...]] input.bag [input2.bag ...]' % sys.argv[0])
    topics = args.topic.split(',') if args.topic != '' else None

    if not os.path.exists(args.outfolder):
        os.mkdir(args.outfolder)
    for infile in infiles:
        with rosbag.Bag(infile) as inbag:
            print ("-------------------- Input: %s ------------------------------" % infile)
            print (inbag)
            outfile = args.outfolder + '/' + infile
            outbag = rosbag.Bag(outfile, 'w')
            msgs = {}
            counts = {}
            for topic,msg,t in inbag.read_messages():
                if topics is not None and topic not in topics and topic.lstrip('/') not in topics:
                    outbag.write(topic, msg, t)
                    continue
                stamp = msg.header.stamp if hasattr(msg, 'header') else t
                if topic in msgs and stamp in msgs[topic]:
                    if msg == msgs[topic][stamp]:
                        counts.setdefault(topic, 0)
                        counts[topic] += 1
                    elif hasattr(msg, 'pose') and msg.pose == msgs[topic][stamp].pose:
                        counts.setdefault(topic, 0)
                        counts[topic] += 1
                    elif hasattr(msg, 'pose') and msg.pose.pose == msgs[topic][stamp].pose.pose:
                        counts.setdefault(topic, 0)
                        counts[topic] += 1
                    else:
                        print (msgs[topic][stamp])
                        print (msg)
                        print ('different message with identical stamp on %s!' % topic)
                        exit()
                else:
                    outbag.write(topic, msg, t)
                    msgs.setdefault(topic, {})[stamp] = msg
            outbag.close()
            outbag = rosbag.Bag(outfile)
            print ("------------------- Output: %s ------------------------------" % outfile)
            print (outbag)
            for topic in counts:
                print ('removed %d duplicates on %s' % (counts[topic], topic))

if __name__ == '__main__':
    main()

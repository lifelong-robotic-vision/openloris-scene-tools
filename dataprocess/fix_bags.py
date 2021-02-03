#!/usr/bin/env python2

# Copyright (C) <2020> Intel Corporation
# SPDX-License-Identifier: MIT

"""
Fix known issues in the released ROS bags of the OpenLORIS-Scene datasets
"""
from __future__ import print_function
import rosbag
import tf
import sys
import os
import argparse
import math

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--outfolder', type=str, default='fixed_bags', \
        help='path to save the processed bags, which will have the same filenames as the original')
    parser.add_argument('-r', '--recursive', action='store_true', \
        help='recursively find all bag files in the given folder and subfolders')
    parser.add_argument('--replace', action='store_true', \
        help='replace the original bags in-place')
    parser.add_argument('-d', '--dry-run', action='store_true', \
        help='go through specified bags without real processing')
    parser.add_argument('-v', '--verbose', action='store_true', \
        help='print bag info')
    parser.set_defaults(recursive=False, replace=False, dry_run=False, verbose=False)

    parser.add_argument('--fix-odom-twist', action='store_true', \
        help='fix the incorrect linear velocities (non-zero y values) on the /odom topic')
    parser.add_argument('--fix-imu-info', action='store_true', \
        help='fix the format and values on /{d400,t265}/{accel,gyro}/imu_info topics')
    parser.add_argument('--fix-home-gt', action='store_true', \
        help='remove the incorrect ground-truth values at the beginning of home1-1')
    parser.add_argument('--fix-all', action='store_true', \
        help='fix all known issues (recommended)')
    parser.set_defaults(fix_odom_twist=False, fix_imu_info=False, fix_home_gt=False, fix_all=False)

    args, left = parser.parse_known_args()
    infiles = []
    for arg in left:
        if os.path.isdir(arg):
            if args.recursive:
                for root, _, files in os.walk(arg):
                    infiles += [os.path.join(root, file) for file in files if file.endswith('.bag')]
            else:
                infiles += [os.path.join(arg, bag) for bag in os.listdir(arg) if bag.endswith('.bag')]
        elif os.path.exists(arg) and arg.endswith('.bag'):
            infiles.append(arg)
        else:
            exit('Error: Invalid argument or filename: ' + arg + '\n(add "-h" to show usages)')
    if len(infiles) == 0:
        print('Usage: %s [args (-h for help)] bag_files_or_folders' % sys.argv[0])
        print('Examples:')
        print('  ' + sys.argv[0] + ' office1-1.bag')
        print('  ' + sys.argv[0] + ' office*.bag')
        print('  ' + sys.argv[0] + ' openloris/office/')
        print('  ' + sys.argv[0] + ' -r openloris/')

    if args.fix_all:
        args.fix_odom_twist = True
        args.fix_imu_info = True
        args.fix_home_gt = True

    if not (args.fix_odom_twist or args.fix_imu_info or args.fix_home_gt):
        parser.print_help()
        print('\nError: Please specify at least one --fix-* argument')
        exit()

    if args.fix_imu_info:
        try:
            from realsense2_camera.msg import IMUInfo
        except ImportError:
            print('Error: --fix-imu-info is specified, but cannot import realsense2_camera.msg.IMUInfo')
            print('Please install realsense-ros and source its setup.bash properly')
            exit()

    if not os.path.exists(args.outfolder):
        os.mkdir(args.outfolder)
    for fileid, infile in enumerate(infiles):
        with rosbag.Bag(infile) as inbag:
            print('- Input [%d/%d]: %s ' % (fileid + 1, len(infiles), infile) + '-' * (60 - len(infile)))
            if args.verbose: print(inbag)
            if args.dry_run: continue
            total = inbag.get_message_count()
            count = 0
            outfile = args.outfolder + '/' + os.path.basename(infile)
            with rosbag.Bag(outfile, 'w') as outbag:
                for topic,msg,t in inbag.read_messages():
                    if topic == '/odom' and args.fix_odom_twist:
                        outbag.write(topic, fix_odom_twist(msg), t)
                    elif topic.endswith('/imu_info') and args.fix_imu_info:
                        outbag.write(topic, fix_imu_info(msg, topic), t)
                    elif topic == '/gt' and args.fix_home_gt and 'home' in infile:
                        if keep_home_gt(t):
                            outbag.write(topic, msg, t)
                    else:
                        outbag.write(topic, msg, t)
                    count += 1
                    if (count % 100 == 0 or count == total):
                        print('Processed %d / %d (%.0f%%)' % (count, total, (float(count) / total * 100.)), end='\r')
            print('\n Output [%d/%d]: %s ' % (fileid, len(infiles), outfile) + '-' * (60 - len(outfile)))
            if args.verbose:
                with rosbag.Bag(outfile) as outbag:
                    print(outbag)
        if args.replace:
            print('mv %s %s' % (outfile, infile))
            os.remove(infile)
            os.rename(outfile, infile)

def fix_odom_twist(msg):
    linear = msg.twist.twist.linear
    if linear.y == 0:
        # already fixed
        return msg
    orientation = msg.pose.pose.orientation
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = tf.transformations.euler_from_quaternion(q)
    if max(abs(euler[0]), abs(euler[1])) > 1e-10:
        print(msg)
        exit('Error: Invalid Euler angles: (%f, %f, %f)' % (euler[0], euler[1], euler[2]))
    x_fixed = linear.x / math.cos(euler[2])
    #print('(%f,%f) | %f | %f' % (linear.x, linear.y, linear.y / math.sin(euler[2]), x_fixed))
    if abs(linear.y / math.sin(euler[2]) - x_fixed) > 1e-10:
        print(msg)
        exit('Error: Inconsistency between orientation and linear velocity')
    linear.x = x_fixed
    linear.y = 0
    return msg

def fix_imu_info(msg, topic):
    # 1. convert format to the version defined in the system
    from realsense2_camera.msg import IMUInfo
    msg_new = IMUInfo()
    if hasattr(msg_new, 'frame_id'):
        if hasattr(msg, 'header'):
            msg_new.frame_id = msg.header.frame_id
        elif hasattr(msg, 'frame_id'):
            msg_new.frame_id = msg.frame_id
        else:
            print(msg)
            exit('Error: Unknown message format on ' + topic)
    elif hasattr(msg_new, 'header'):
        if hasattr(msg, 'header'):
            msg_new.header = msg.header
        elif hasattr(msg, 'frame_id'):
            msg_new.header.frame_id = msg.frame_id
        else:
            print(msg)
            exit('Error: Unknown message format on ' + topic)
    else:
        print(msg)
        exit('Error: Unsupported IMUInfo format -- check your realsense-ros version and submit an issue to OpenLORIS-Scene')
    msg_new.data = msg.data
    msg_new.noise_variances = msg.noise_variances
    msg_new.bias_variances = msg.bias_variances

    # 2. add values of D435 IMU variances (scaled from values of T265)
    if msg_new.noise_variances[0] == 0:
        if topic == '/d400/accel/imu_info':
            noise = 0.00026780980988405645  # 6.695245247101411e-05 * 4
            bias = 2.499999936844688e-05    # 9.999999747378752e-05 / 4
        elif topic == '/d400/gyro/imu_info':
            noise = 1.0296060281689279e-05  # 5.148030140844639e-06 * 2
            bias = 2.499999993688107e-07    # 4.999999987376214e-07 / 2
        else:
            exit('Error: Unknown im_info topic: ' + topic)
        msg_new.noise_variances = [noise] * 3
        msg_new.bias_variances = [bias] * 3
    return msg_new

def keep_home_gt(t):
    return not t.to_sec() in (\
        1560000002.6138489, 1560000002.6441183, 1560000002.7450459,
        1560000002.8459241, 1560000002.8761814, 1560000002.8862779,
        1560000002.9770176, 1560000002.9770176, 1560000002.9870765)

if __name__ == '__main__':
    main()

#!/usr/bin/env python2
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

    parser.add_argument('--fix-odom-twist', action='store_true', \
        help='fix the incorrect linear velocities (non-zero y values) on the /odom topic')

    parser.set_defaults(recursive=False, replace=False, dry_run=False, verbose=False, fix_odom_twist=True)

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
            exit('Invalid argument or filename: ' + arg + '\n(add "-h" to show usages)')
    if len(infiles) == 0:
        print('Usage: %s [args (-h for help)] bag_files_or_folders' % sys.argv[0])
        print('Examples:')
        print('  ' + sys.argv[0] + ' office1-1.bag')
        print('  ' + sys.argv[0] + ' office*.bag')
        print('  ' + sys.argv[0] + ' openloris/office/')
        print('  ' + sys.argv[0] + ' -r openloris/')

    if not os.path.exists(args.outfolder):
        os.mkdir(args.outfolder)
    for infile in infiles:
        with rosbag.Bag(infile) as inbag:
            print('-' * 21 + ' Input: %s ' % infile + '-' * (40 - len(infile)))
            if args.verbose: print(inbag)
            if args.dry_run: continue
            total = inbag.get_message_count()
            count = 0
            outfile = args.outfolder + '/' + os.path.basename(infile)
            with rosbag.Bag(outfile, 'w') as outbag:
                for topic,msg,t in inbag.read_messages():
                    if args.fix_odom_twist and topic == '/odom':
                        outbag.write(topic, fix_odom_twist(msg), t)
                    else:
                        outbag.write(topic, msg, t)
                    count += 1
                    if (count % 100 == 0 or count == total):
                        print('processing %d / %d (%.0f%%)' % (count, total, float(count) / total * 100), end='\r')
            print('\n' + '-' * 20 + ' Output: %s ' % outfile + '-' * (40 - len(outfile)))
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
        exit('Error: invalid Euler angles: (%f, %f, %f)' % (euler[0], euler[1], euler[2]))
    x_fixed = linear.x / math.cos(euler[2])
    #print('(%f,%f) | %f | %f' % (linear.x, linear.y, linear.y / math.sin(euler[2]), x_fixed))
    if abs(linear.y / math.sin(euler[2]) - x_fixed) > 1e-10:
        print(msg)
        exit('Error: inconsistency between orientation and linear velocity')
    linear.x = x_fixed
    linear.y = 0
    return msg

if __name__ == '__main__':
    main()

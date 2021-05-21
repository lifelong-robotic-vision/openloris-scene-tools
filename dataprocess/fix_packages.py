#!/usr/bin/env python

# Copyright (C) <2020-2021> Intel Corporation
# SPDX-License-Identifier: MIT
# Author: Xuesong Shi

"""
Fix known issues in the released packages of the OpenLORIS-Scene datasets
"""
from __future__ import print_function
import sys
import os
import argparse
import math
import subprocess
tf_imported = False
try:
    from tf import transformations
    tf_imported = True
except ImportError as e:
    print(e)
if not tf_imported:
    try:
        import transformations
        tf_imported = True
    except ImportError as e:
        print(e)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--recursive', action='store_true', \
        help='recursively find all 7z/odom/sensors/groundtruth files in the given folder and subfolders')
    parser.add_argument('-d', '--dry-run', action='store_true', \
        help='go through specified files without real processing')
    parser.set_defaults(recursive=False, dry_run=False)

    parser.add_argument('--fix-odom-twist', action='store_true', \
        help='fix the incorrect linear velocities (non-zero y values) in odom.txt')
    parser.add_argument('--fix-imu-info', action='store_true', \
        help='fix the noise and bias variances of d400_{accelerometer,gyroscope} in sensors.yaml')
    parser.add_argument('--fix-home-gt', action='store_true', \
        help='remove the incorrect ground-truth values at the beginning of home1-1')
    parser.add_argument('--fix-all', action='store_true', \
        help='fix all known issues (recommended)')
    parser.set_defaults(fix_odom_twist=False, fix_imu_info=False, fix_home_gt=False, fix_all=False)

    args, left = parser.parse_known_args()
    infiles_7z = []
    infiles_odom = []
    infiles_sensors = []
    infiles_gt = []
    for arg in left:
        if os.path.isdir(arg):
            if args.recursive:
                for root, _, files in os.walk(arg):
                    infiles_7z += [os.path.join(root, file) for file in files if file.endswith('.7z')]
                    infiles_odom += [os.path.join(root, file) for file in files if file == 'odom.txt']
                    infiles_sensors += [os.path.join(root, file) for file in files if file == 'sensors.yaml']
                    infiles_gt += [os.path.join(root, file) for file in files if file == 'groundtruth.txt']
            else:
                infiles_7z += [os.path.join(arg, file) for file in os.listdir(arg) if file.endswith('.7z')]
                infiles_odom += [os.path.join(arg, file) for file in os.listdir(arg) if file == 'odom.txt']
                infiles_sensors += [os.path.join(arg, file) for file in os.listdir(arg) if file == 'sensors.yaml']
                infiles_gt += [os.path.join(arg, file) for file in os.listdir(arg) if file == 'groundtruth.txt']
        elif os.path.exists(arg) and arg.endswith('.7z'):
            infiles_7z.append(arg)
        else:
            exit('Invalid argument or filename: ' + arg + '\n(add "-h" to show usages)')
    if len(infiles_7z) + len(infiles_odom) + len(infiles_sensors) == 0:
        print('Usage: %s [args (-h for help)] files_or_folders' % sys.argv[0])
        print('Examples:')
        print('  ' + sys.argv[0] + ' --fix-all office1-1.7z')
        print('  ' + sys.argv[0] + ' --fix-all office1-1/')
        print('  ' + sys.argv[0] + ' --fix-all -r openloris/')
        print('\nError: Find no 7z or extracted files to fix. Refer to the examples above.')
        exit()

    if args.fix_all:
        args.fix_odom_twist = True
        args.fix_imu_info = True
        args.fix_home_gt = True

    if not (args.fix_odom_twist or args.fix_imu_info or args.fix_home_gt):
        parser.print_help()
        print('\nError: Please specify at least one --fix-* argument')
        exit()

    if args.fix_odom_twist and not tf_imported:
        print('Error: Cannot import tf nor transformations which is required for --fix-odom-twist')
        print('Please install/activate one of them either by `source /opt/ros/VERSION/setup.bash` or `pip install --user transformations`')
        exit()

    if args.fix_odom_twist:
        for infile in infiles_odom:
            if 'market' in infile: continue
            print('Processing %s ' % infile + '-' * (60 - len(infile)))
            if args.dry_run: continue
            fix_odom(infile)

    if args.fix_imu_info:
        for infile in infiles_sensors:
            print('Processing %s ' % infile + '-' * (60 - len(infile)))
            if args.dry_run: continue
            fix_sensors(infile)

    if args.fix_home_gt:
        for infile in infiles_gt:
            if 'home' not in infile: continue
            print('Processing %s ' % infile + '-' * (60 - len(infile)))
            if args.dry_run: continue
            fix_home_gt(infile)

    for infile in infiles_7z:
        if not (
                (args.fix_odom_twist and 'market' not in infile) or
                (args.fix_imu_info) or
                (args.fix_home_gt and 'home' in infile)
                ):
            continue
        print('Processing %s ' % infile + '-' * (60 - len(infile)))
        if args.dry_run: continue
        path_in_7z = os.path.basename(infile)[:-3]
        odom_in_7z = path_in_7z + '/odom.txt'
        sensors_in_7z = path_in_7z + '/sensors.yaml'
        gt_in_7z = path_in_7z + '/groundtruth.txt'
        subprocess.call(['7z', 'x', '-aoa', '-bso0', infile, odom_in_7z, sensors_in_7z, gt_in_7z])
        if not os.path.exists(odom_in_7z) or not os.path.exists(sensors_in_7z) or not os.path.exists(gt_in_7z):
            exit('Error: Failed to decompress %s' % (infile))
        fix_odom(odom_in_7z)
        fix_sensors(sensors_in_7z)
        if 'home' in infile: fix_home_gt(gt_in_7z)
        subprocess.call(['7z', 'u', '-bso0', infile, odom_in_7z, sensors_in_7z, gt_in_7z])
        os.remove(odom_in_7z)
        os.remove(sensors_in_7z)
        os.remove(gt_in_7z)
        if len(os.listdir(path_in_7z)) == 0:
            os.removedirs(path_in_7z)


def fix_odom(filename):
    outfilename = filename + '.tmp'
    with open(filename) as infile, open(outfilename, 'w') as outfile:
        lines = infile.readlines()
        for line in lines:
            if line.startswith('#') or len(line.strip()) == 0:
                outfile.write(line)
                continue
            splits = line.split()
            if len(splits) != 14:
                print(line)
                exit('Error: Invalid odom data in ' + filename)
            q = [float(v) for v in splits[4:8]]
            linear = [float(v) for v in splits[8:11]]
            if linear[1] == 0:
                # already fixed
                outfile.write(line)
                continue
            euler = transformations.euler_from_quaternion(q)
            if max(abs(euler[0]), abs(euler[1])) > 1e-10:
                print(line)
                exit('Error: Invalid Euler angles: (%f, %f, %f)' % (euler[0], euler[1], euler[2]))
            x_fixed = linear[0] / math.cos(euler[2])
            #print('(%f,%f) | %f | %f' % (linear.x, linear.y, linear.y / math.sin(euler[2]), x_fixed))
            if abs(linear[1] / math.sin(euler[2]) - x_fixed) > 1e-10:
                print(line)
                exit('Error: Inconsistency between orientation and linear velocity')
            splits[8] = '%.12f' % (x_fixed)
            splits[9] = '0.0'
            outfile.write(' '.join(splits) + '\n')
    os.remove(filename)
    os.rename(outfilename, filename)

def fix_sensors(filename):
    outfilename = filename + '.tmp'
    with open(filename) as infile:
        lines = infile.readlines()
        empty_data = '      data: [ 0., 0., 0. ]\n'
        # d400_accelerometer.noise_variances
        if lines[97] == empty_data:
            lines[97] = '      data: [ 0.00026780980988405645, 0.00026780980988405645, 0.00026780980988405645 ]\n'
        # d400_accelerometer.bias_variances
        if lines[102] == empty_data:
            lines[102] = '      data: [ 2.499999936844688e-05, 2.499999936844688e-05, 2.499999936844688e-05 ]\n'
        # d400_gyroscope.noise_variances
        if lines[117] == empty_data:
            lines[117] = '      data: [ 1.0296060281689279e-05, 1.0296060281689279e-05, 1.0296060281689279e-05 ]\n'
        # d400_gyroscope.bias_variances
        if lines[122] == empty_data:
            lines[122] = '      data: [ 2.499999993688107e-07, 2.499999993688107e-07, 2.499999993688107e-07 ]\n'
    with open(outfilename, 'w') as outfile:
        outfile.writelines(lines)
    os.remove(filename)
    os.rename(outfilename, filename)

def fix_home_gt(filename):
    invalid_stamps = (\
        '1560000002.61384892', '1560000002.64411831', '1560000002.74504590',
        '1560000002.84592414', '1560000002.87618136', '1560000002.88627791',
        '1560000002.97701764', '1560000002.97701764', '1560000002.98707652')
    with open(filename) as infile:
        lines = infile.readlines()
        if lines[1].split()[0] not in invalid_stamps:
            return
    with open(filename, 'w') as outfile:
        for line in lines:
            splits = line.split()
            if len(splits) > 0 and splits[0] in invalid_stamps: continue
            outfile.write(line)

if __name__ == '__main__':
    main()

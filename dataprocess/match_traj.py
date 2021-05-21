#!/usr/bin/env python2

# Copyright (C) <2019> Intel Corporation
# SPDX-License-Identifier: MIT
# Authors: Zhigang Wang; Xuesong Shi

import rosbag
import math
import matplotlib.pyplot as plt
from tum_evaluate_tools import associate
from tum_evaluate_tools import evaluate_ate
import numpy
import argparse
import sys
import segway_transforms

def search_time_diff(ref_traj, target_traj, t_min, t_max, precision, ax_ate, ax_traj, plot_axis):
    """
    return offset, match_count, ate
    """
    results = {} # format: { offset: (ate, match_count, rot, trans) }
    while t_max - t_min > precision:
        step = (t_max - t_min) / 4
        for offset in range(t_min, t_max + step, step):
            if offset not in results:
                if args.interpolation:
                    matches, ref_traj_interpolated = associate.associate_with_interpolation(ref_traj, target_traj, offset, 30000000)
                else:
                    ref_traj_interpolated = ref_traj
                    matches = associate.associate(ref_traj_interpolated, target_traj,offset,5000000)
                if len(matches)<2:
                    print ("Couldn't find matching timestamp pairs!")
                    t1max = max(ref_traj.keys())#, key=(lambda k: float(ref_traj[k])))
                    t1min = min(ref_traj.keys())#, key=(lambda k: float(ref_traj[k])))
                    t2max = max(target_traj.keys())#, key=(lambda k: float(ref_traj[k])))
                    t2min = min(target_traj.keys())#, key=(lambda k: float(ref_traj[k])))
                    print ('traj 1: %19d -> %19d (%d msgs)' % (t1min, t1max, len(ref_traj)))
                    print ('traj 2: %19d -> %19d (%d msgs)' % (t2min, t2max, len(target_traj)))
                    exit()

                first_xyz = numpy.matrix([[float(value) for value in ref_traj_interpolated[a][0:3]] for a,b in matches]).transpose()
                second_xyz = numpy.matrix([[float(value) for value in target_traj[b][0:3]] for a,b in matches]).transpose()
                rot,trans,trans_error = evaluate_ate.align(second_xyz,first_xyz)
                ate = numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
                results[offset] = (ate, len(matches), rot, trans)
                print("ATE for offset %f (ms): %f (%d matches)" % (offset * 1e-6, ate, len(matches)))
        t_optimal = sorted(results.items(), key = lambda x : x[1][0])[0][0]
        offsets = sorted(results)
        opt_id = offsets.index(t_optimal)
        t_min = t_optimal - step * 4 if opt_id == 0 else offsets[opt_id - 1]
        t_max = t_optimal + step * 4 if opt_id == len(offsets) - 1 else offsets[opt_id + 1]
        print ('optimal offset range %f - %f - %f (ms)' % (t_min * 1e-6, t_optimal * 1e-6, t_max * 1e-6))

    #traj = { t : (rot * target_traj[t] + trans) for t in target_traj}
    ax_ate.plot([t * 1e-6 for t in sorted(results)], [results[offset][0] for offset in sorted(results)], '.-')
    stamps = sorted(target_traj.keys())
    traj = rot * numpy.matrix([[float(value) for value in target_traj[t][0:3]] for t in stamps]).transpose() + trans
    ax_traj.plot(list((numpy.array(stamps) + t_optimal) * 1e-9), list(traj.A[plot_axis]), '-')
    return t_optimal, results[t_optimal][1], results[t_optimal][0]

def msg_to_pose_values(msg):
    # PoseStamped
    if hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):
        t = msg.pose.position
        q = msg.pose.orientation
    # Odometry / PoseWithCovarianceStamped
    elif hasattr(msg, 'pose') and hasattr(msg.pose, 'pose') and hasattr(msg.pose.pose, 'position'):
        t = msg.pose.pose.position
        q = msg.pose.pose.orientation
    # tf
    elif hasattr(msg, 'translation'):
        t = msg.translation
        q = msg.rotation
    return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]

def read_traj(bagfile, topic, tmin, tmax):
    traj = {}
    with rosbag.Bag(bagfile) as inbag:
        for topic,msg,t in inbag.read_messages([topic]):
            if not hasattr(msg, 'header'):
                print (msg)
                exit('No header on topic %s in %s' % (topic, bagfile))
            ts = msg.header.stamp.to_nsec()
            if (ts * 1e-9 < tmin): continue
            if (ts * 1e-9 > tmax): break
            traj[ts - tmin] = msg_to_pose_values(msg)
    return traj

def read_t265_traj(bagfile, tmin, tmax):
    traj = {}
    with rosbag.Bag(bagfile) as inbag:
        for topic, msg, t in inbag.read_messages(topics=['/device_0/sensor_0/Pose_0/pose/metadata']):
            if msg.key == 'frame_timestamp':
                offset = int(float(msg.value) * 1e6) - t.to_nsec()
                print ('T265 time offset: %d' % (offset))
                break
        for topic, msg, t in inbag.read_messages(topics=['/device_0/sensor_0/Pose_0/pose/transform/data']):
            ts = t.to_nsec() + offset
            if (ts * 1e-9 < tmin): continue
            if (ts * 1e-9 > tmax): break
            ts = ts - tmin
            traj[ts] = msg_to_pose_values(msg)
            # workaround to use (x,z,0) instead of (x,y,z)
            if not args.transform:
                traj[ts][1] = traj[ts][2]
                traj[ts][2] = 0
    return traj

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='''
    This script is use to caculate the time stampe difference between different SLAM related sensors.
    ''')
    parser.add_argument('--ref', default='t265')
    parser.add_argument('--mocap-bag', help='rosbag file which include mcs pose information', default='preprocessed.bag' )
    parser.add_argument('--odom-bag', help='rosbag file which include odometry information', default='preprocessed.bag' )
    parser.add_argument('--t265-bag', help='rosbag file which include t265 transform information', default='record-t265.bag')
    parser.add_argument('--laser-bag', help='rosbag file which include laser slam pose information', default='hector-pose.bag')
    parser.add_argument('--min-time-diff', type=float, help='Initial lower bound of time difference', default=-0.1)
    parser.add_argument('--max-time-diff', type=float, help='Initial upper bound of time difference', default=0.1)
    parser.add_argument('--search-step', type=float, help='The mininal step of time difference searching', default=0.001)
    parser.add_argument('--start-time', type=float, help='start_time and end_time is used for cutting out the data used for analysis', default=0)
    parser.add_argument('--end-time', type=float, help='start_time and end_time is used for cutting out the data used for analysis', default=40)
    parser.add_argument('--output-file', type=str, help='output file', default='./sync.yaml')
    parser.add_argument('--interpolation', type=int, help='interpolate the reference trajectory', default=1)
    parser.add_argument('--transform', type=int, help='transform trajectories to the same target frame', default=1)
    args = parser.parse_args()
    res_str = '#' + ' '.join(sys.argv) + '\n# args: ' + str(args)

    # format: {name: (bag, topic, [(tf_parent, tf_child)], frame)}
    traj_config = {}
    if args.mocap_bag is not '':
        traj_config['mocap'] = (args.mocap_bag, ['/vrpn_client_node/RigidBody1/pose'], [('world', 'RigidBody1')], 'marker')
    if args.laser_bag is not '':
        if 'gaussian' in args.laser_bag:
            traj_config['laser'] = (args.laser_bag, ['/v5_current_pose', '/scan'], [], 'base_link')
        else:
            traj_config['laser'] = (args.laser_bag, ['/slam_out_pose', '/scan'], [], 'laser')
    traj_config['odom'] = (args.odom_bag, ['/odom'], [('base_odom', 'base_link')], 'base_link')
    traj_config['t265'] = (args.t265_bag, ['/device_0/sensor_0/Pose_0/pose/transform/data'], [], 't265_pose')
    #traj_config['d400'] = ('orb-pose.bag', ['/maslam/pose'], [], 'd400_color')

    ref = args.ref
    tmin = args.start_time
    tmax = args.end_time
    # convert relative time to absolute time
    with rosbag.Bag(traj_config['odom'][0]) as refbag:
        tstart = refbag.get_start_time()
        tend = refbag.get_end_time()
    if (tmin != 0 and tmin < 1e9) or (tmax < 1e9): # relative time
        rmin = tmin
        rmax = tmax
        tmin += tstart
        tmax += tstart
    else:
        rmin = tmin - tstart if tmin != 0 else 0
        rmax = tmax - tstart
    res = 'Match traj from %f to %f / %f to %f (%f sec)' % \
        (rmin, rmax, tmin, tmax, min([tmax, tend]) - max([tmin, tstart]))
    print res
    res_str += '# ' + res + '\n'
    res_table = 'start end target ref offset matches RMSE\n'

    trajs = {}
    for name, config in traj_config.items():
        traj = read_traj(config[0], config[1][0], tmin, tmax) if name is not 't265' \
            else read_t265_traj(config[0], tmin, tmax)
        res = '%s: %d points' % (name, len(traj))
        print res
        res_str += '# ' + res + '\n'
        if (len(traj) < 1): exit('Could not find traj of %s' % name)
        if args.transform and name != ref:
            trans_matrix = segway_transforms.get_tf_matrix(config[3], traj_config[ref][3])
            print '------------------\nT(%s -> %s)' % (name, ref)
            print trans_matrix
            transform = lambda pose: segway_transforms.tf_matrix_to_values(numpy.dot(segway_transforms.tf_values_to_matrix(pose), trans_matrix))
            trajs[name] = {t: transform(pose) for t,pose in traj.items()}
        else:
            trajs[name] = traj

    min_time_diff = int(args.min_time_diff * 1e9)
    max_time_diff = int(args.max_time_diff * 1e9)
    search_step = int(args.search_step * 1e9)
    fig = plt.figure()
    ax_ate = fig.add_subplot(211)
    ax_traj = fig.add_subplot(212)
    # decide the principle axis to be ploted
    axis_range_max = 0
    for axis in range(3):
        tmp = [p[axis] for p in trajs[ref].values()]
        axis_range = max(tmp) - min(tmp)
        if axis_range > axis_range_max:
            plot_axis = axis
            axis_range_max = axis_range

    offsets = {}
    legend_ate = []
    legend_traj = []
    for name, config in traj_config.items():
        if name == ref:
            offsets[name] = 0
            continue
        print ("-------------------------")
        print ('matching %s and %s...' % (ref, name))
        offset, match_count, rmse = search_time_diff(trajs[ref], trajs[name], min_time_diff, max_time_diff, search_step, ax_ate, ax_traj, plot_axis)
        res = '%s to %s offset: %f ms (%d matches; RMSE %f)' % (name, ref, offset * 1e-6, match_count, rmse)
        print res
        res_str += '# ' + res + '\n'
        res_table += '%f %f %s %s %f %d %f\n' % (args.start_time, args.end_time, name, ref, offset * 1e-6, match_count, rmse)
        offsets[name] = offset
        legend_ate.append('%s-%s' % (name, ref))
        legend_traj.append(name)

    # print results
    print ("-------------------------")
    print res_table

    # plot results
    stamps = sorted(trajs[ref].keys())
    traj = numpy.matrix([[float(value) for value in trajs[ref][t][0:3]] for t in stamps]).transpose()
    ax_traj.plot(list((numpy.array(stamps)) * 1e-9), list(traj.A[plot_axis]), '-')
    ax_ate.legend(legend_ate)
    ax_ate.set_xlabel('offset (ms)')
    ax_ate.set_ylabel('ATE RMSE (m)')
    legend_traj.append(ref)
    ax_traj.legend(legend_traj)
    ax_traj.set_xlabel('t')
    ax_traj.set_ylabel('x' if plot_axis == 0 else 'y' if plot_axis == 1 else 'z')

    fo = open(args.output_file, 'wt')
    topics = ''
    tfs = ''
    for name, config in traj_config.items():
        # set all offsets against camera, to avoid from processing camera bags
        if name == 't265':
            continue
        offset = (offsets[name] - offsets['t265']) * 1e-9
        for topic in config[1]:
            topics += '  %s: %f\n' % (topic, offset)
        for tf in config[2]:
            tfs += '  %s:\n    %s: %f\n' % (tf[0], tf[1], offset)
    yaml_str = res_str + """
# all time values are in seconds\n\
# the following offsets will be added additionally to correspondent messages\n\
topics:
%s
tf:
%s
""" % (topics, tfs)
    fo.write(yaml_str)
    fo.close()

    plt.show()

#!/usr/bin/env python

# Copyright (C) <2019-2021> Intel Corporation
# SPDX-License-Identifier: MIT
# Authors: Siyuan Lu; Xuesong Shi

from __future__ import print_function

import argparse
from collections import OrderedDict
import glob
import logging
import psutil
#import rosbag
import rospy
import sys
import time
import threading
import subprocess
import yaml
import signal
import os

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from std_msgs.msg import String

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
#formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
formatter = logging.Formatter('%(message)s')
ch = logging.StreamHandler()
ch.setFormatter(formatter)
logger.addHandler(ch)

init_pose_topic = '/openloris/reset_pose'
valid_frames = ['d400_color', 'd400_depth', 'd400_imu', 't265_fisheye1', 't265_fisheye2', 't265_imu', 'base_link']

global pose_count
global current_pose

def pose_callback(data):
    #logger.info(data)
    print("%d.%d %s %f %f %f %f %f %f %f"
            % (data.header.stamp.secs, data.header.stamp.nsecs,
                rospy.get_time(),
                data.pose.position.x,
                data.pose.position.y,
                data.pose.position.z,
                data.pose.orientation.x,
                data.pose.orientation.y,
                data.pose.orientation.z,
                data.pose.orientation.w)
            )
    global pose_count, current_pose
    pose_count += 1
    current_pose = (data.pose.position.x,
                    data.pose.position.y,
                    data.pose.position.z,
                    data.pose.orientation.x,
                    data.pose.orientation.y,
                    data.pose.orientation.z,
                    data.pose.orientation.w)

def handle_listener(topic):
    rospy.Subscriber(topic, PoseStamped, pose_callback)
    rospy.spin()

def sigint_handler(sig, frame):
    print('# interrupted by user')
    sys.exit('\nExit testing...\n')

def play_sequences(bags, topics, aided_reloc, scene, frame, pub_pose):
    seq = 1
    DEVNULL = open(os.devnull, 'r+b', 0)
    global pose_count, current_pose
    for bag in bags:
        if seq != 1:
            if aided_reloc:
                init_pose = PoseStamped()
                #p = get_init_pose(scene, seq, frame)
                p = openloris_init_poses[frame][scene][seq]
                init_pose.header.seq = seq
                init_pose.header.frame_id = frame
                init_pose.pose.position.x = p[0]
                init_pose.pose.position.y = p[1]
                init_pose.pose.position.z = p[2]
                init_pose.pose.orientation.x = p[3]
                init_pose.pose.orientation.y = p[4]
                init_pose.pose.orientation.z = p[5]
                init_pose.pose.orientation.w = p[6]
                pub_pose.publish(init_pose)
                logger.info('Published initial pose for the next sequence on %s' % (init_pose_topic))
            time.sleep(1)
        print("seq: %d" % seq)
        print("aided_reloc: false")
        logger.info('Playing %s ...' % (bag))
        pose_count = 0
        current_pose = None
        process = subprocess.Popen(['rosbag', 'play', bag, '--topics'] + topics, stdin=DEVNULL, stdout=DEVNULL)
        previous_pose = None
        previous_count = 0
        while process.poll() == None:
            if current_pose is not None and current_pose != previous_pose:
                logger.info('Received poses in %d FPS. Current: (%s)' % (pose_count - previous_count, ', '.join('%.4f' % v for v in current_pose)))
                previous_pose = current_pose
                previous_count = pose_count
            time.sleep(1)
        time.sleep(1)
        logger.info('Got %d poses for %s' % (pose_count, bag.split('/')[-1]))
        seq += 1

def publish_msg(msg, pose):
    pub_n = rospy.Publisher('new_sequence', UInt8, queue_size=10)
    pub_p = rospy.Publisher('/openloris/reset_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    time = rospy.get_rostime().to_sec() + 1
    while True:
        new_time = rospy.get_rostime().to_sec()
        if new_time > time:
            return
        pub_n.publish(msg)
        rate.sleep()
    pub_p.publish(pose)

def cpu_info():
    CPUinfo=OrderedDict()
    procinfo=OrderedDict()
    nprocs = 0
    with open('/proc/cpuinfo') as f:
        for line in f:
            if not line.strip():
                #end of one processor
                CPUinfo['proc%s' % nprocs]=procinfo
                nprocs = nprocs+1
                #Reset
                procinfo=OrderedDict()
            else:
                if len(line.split(':')) == 2:
                    procinfo[line.split(':')[0].strip()] = line.split(':')[1].strip()
                else:
                    procinfo[line.split(':')[0].strip()] = ''
    return CPUinfo

def complete_topics(topics):
    out = []
    for topic in topics:
        topic = topic.strip()
        if not topic.startswith('/'):
            topic = '/' + topic
        topic = topic.rstrip('/')
        if topic.endswith('color') or topic.endswith('depth') or topic.endswith('fisheye1') or topic.endswith('fisheye2'):
            out.append(topic + '/image_raw')
            out.append(topic + '/camera_info')
        elif topic.endswith('accel') or topic.endswith('gyro'):
            out.append(topic + '/sample')
            out.append(topic + '/imu_info')
        else:
            out.append(topic)
    return out

def generate_info(algorithm, topics, target_frame, use_gpu):
    info = ''
    info += 'algorithm: %s\n' % algorithm
    info += 'topics: %s\n' % ','.join(topics)
    info += 'frame: %s\n' % target_frame
    for processor in cpu_info().keys():
        cpu_model = cpu_info()[processor]['model name']
    info += 'CPU: %s\n' % (cpu_model)
    if use_gpu:
        try:
            import pynvml
            has_pynvml = True
        except ModuleNotFoundError:
            print("Install pynvml to get GPU info")
            has_pynvml = False
        if has_pynvml:
            info += 'GPU:'
            pynvml.nvmlInit()
            deviceCount = pynvml.nvmlDeviceGetCount()
            for i in range(deviceCount):
                handle = pynvml.nvmlDeviceGetHandleByIndex(i)
                gpu = pynvml.nvmlDeviceGetName(handle)
                info += ' ' + gpu.decode('utf-8')
            info += '\n'
        else:
            info += 'GPU: unable to detect; please install pynvml\n'
    mem_g = psutil.virtual_memory().total/1024/1024/1024
    info += "memory: %d GB\n" % mem_g
    return info

def get_bag_list(path, scene, sequences):
    bags = []
    for seq in range(1, sequences + 1):
        results = glob.glob(r'%s/%s-1-%d*.bag' % (path, scene, seq))
        if len(results) < 1: sys.exit('Cannot find %s-1-%d*.bag in %s' % (scene, seq, path))
        bags.append(results[0])
    return bags

def main():
    '''
        Main function of this script.
    '''
    #logger.info('Please check the above testing configurations. Correct? [Y/n]')
    #raw_input()
    # Parse config
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--config', type=str, default='test.yaml', help='config file')
    args = parser.parse_args()

    config_file = args.config
    logger.info('config: %s', config_file)
    config = None
    with open(config_file) as cfile:
        config = yaml.load(cfile)
    if config is None:
        sys.exit('Could not open the configuration file %s' % config_file)
    #print(config)
    topics = complete_topics(config['input_topics'])
    rate = config['rate'] if 'rate' in config else 1
    tests = []
    for test in config['tests']:
        if test['enable']:
            test['bags'] = get_bag_list(test['path'], test['scene'], test['sequences'])
            tests.append(test)
    pub_pose = rospy.Publisher(init_pose_topic, PoseStamped, queue_size=10)

    # Confirm config
    if config['frame'] not in valid_frames:
        logger.error('Invalid frame %s' % config['frame'])
        logger.error('Valid frames are: %s', ', '.join(valid_frames))
        sys.exit()
    info = generate_info(config['algorithm'], config['input_topics'], config['frame'], config['use_gpu'])
    logger.info('---------------------------------------------')
    logger.info(info)
    #for item in ['algorithm', 'pose_topic', 'frame', 'aided_reloc', 'use_gpu', 'input_topics']:
    #    logger.info('%s: %s' % (item, str(configs[item])))
    for test in tests:
        logger.info('--\nscene: %s' % test['scene'])
        logger.info('  ' + '\n  '.join(test['bags']))
    logger.info('---------------------------------------------')
    key = raw_input('Please check the above testing configurations. Correct? [Y/n]')
    if key not in ['', 'y', 'Y']:
        logger.info('Please correct the configurations in %s' % config_file)
        sys.exit()

    # Prepare for testing
    rospy.init_node('openloris_test', anonymous=True)
    t_listener = threading.Thread(target=handle_listener, args=(config['pose_topic'],))
    t_listener.setDaemon(True)
    t_listener.start()
    outfolder = 'openloris_results_%s' % config['algorithm']
    if not os.path.exists(outfolder):
        os.mkdir(outfolder)

    signal.signal(signal.SIGINT, sigint_handler)

    # Test
    for test in tests:
        scene = test['scene']
        logger.info('---------------------------------------------')
        logger.info('Test: %s (%d sequences)' % (scene, test['sequences']))
        logger.info('Please %s your algorithm, make sure it has subscribed the right topics, and will publish poses (PoseStamped) on %s' \
            % ('get ready' if tests.index(test) == 0 else 'restart/reset', config['pose_topic']))
        logger.info('Once ready, you are not allowed to provide any manual input to your algorithm. <Press Enter to start>')
        raw_input()
        outfilename = outfolder + ('/%s-1.txt' % scene)
        sys.stdout = open(outfilename, mode='w')
        print('scene: %s' % scene)
        print(info)
#        try:
        play_sequences(test['bags'], topics, config['aided_reloc'], scene, config['frame'], pub_pose)
        sys.stdout.close()
        logger.info('Results has been saved to %s' % outfilename)
#        except Exception as e:
#            logger.error(e)
#            sys.exit(-1)

openloris_init_poses = {
    "d400_color": {
        "home": {
            1: (0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000),
            2: (2.048979,-0.002665,4.537484,-0.009075,-0.562620,0.003767,0.826657),
            3: (-0.768389,0.061032,7.263491,-0.010878,-0.674418,0.004516,0.738255),
            4: (-2.449708,0.068256,4.292432,0.007224,0.447841,-0.002999,0.894079),
            5: (0.920740,-0.006231,1.287274,0.012846,0.796441,-0.005333,0.604556),
        },
        "market": {
            1: (-0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,1.000000),
            2: (1.305539,-2.203308,15.508237,-0.000171,0.059580,0.008479,0.998187),
            3: (1.082180,-2.558096,17.996719,0.000065,-0.022770,-0.003241,0.999735),
        },
        "cafe": {
            1: (0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000),
            2: (-0.580684,0.010388,0.152612,-0.002699,-0.167305,0.001120,0.985901),
        },
        "corridor": {
            1: (0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000),
            2: (-36.176285,0.646324,9.377665,-0.001365,-0.084694,0.000567,0.996406),
            3: (-36.552540,0.654303,9.662947,-0.001386,-0.085934,0.000575,0.996300),
            4: (-46.958865,0.900254,21.325972,0.012293,0.762042,-0.005103,0.647391),
            5: (-36.054248,0.629627,7.177968,-0.001892,-0.117289,0.000785,0.993096),
        },
        "office": {
            1: (0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000),
            2: (-3.442015,0.003196,0.338312,-0.005375,0.120520,0.000077,0.992696),
            3: (-3.694180,-0.012357,-0.251014,-0.001239,0.999841,-0.017416,0.003740),
            4: (-3.552656,-0.004217,-0.019779,-0.001152,0.989872,-0.015916,0.141061),
            5: (-3.530060,0.012260,0.878919,0.003576,0.033667,0.000285,0.999427),
            6: (0.011558,-0.000277,-0.099034,-0.004520,0.024910,-0.000547,0.999679),
            7: (-3.505344,0.010941,0.707510,-0.008124,0.198803,0.000582,0.980006),
        },
    },
    "d400_depth": {
        "home": {
            1: (0.000000,0.000000,0.000000,-0.000000,0.000000,0.000000,1.000000),
            2: (2.012263,0.021269,4.562794,-0.006943,-0.562634,0.005684,0.826657),
            3: (-0.825435,0.083665,7.272808,-0.008322,-0.674436,0.006814,0.738255),
            4: (-2.480525,0.073705,4.265565,0.005526,0.447852,-0.004525,0.894079),
            5: (0.894962,0.001863,1.277470,0.009828,0.796461,-0.008047,0.604556),
        },
        "market": {
            1: (-0.000000,0.000000,-0.000000,-0.000000,0.000000,-0.000000,1.000000),
            2: (1.179254,-2.180837,15.519733,-0.000560,0.059587,0.008416,0.998187),
            3: (0.935694,-2.534688,18.008920,0.000214,-0.022773,-0.003216,0.999735),
        },
        "cafe": {
            1: (0.000000,0.000000,0.000000,-0.000000,0.000000,0.000000,1.000000),
            2: (-0.582661,0.008746,0.154042,-0.002065,-0.167310,0.001690,0.985901),
        },
        "corridor": {
            1: (0.000000,0.000000,0.000000,-0.000000,0.000000,0.000000,1.000000),
            2: (-36.233228,0.539726,9.165883,-0.001044,-0.084696,0.000856,0.996406),
            3: (-36.611187,0.547249,9.448964,-0.001060,-0.085936,0.000868,0.996300),
            4: (-47.102917,0.793742,21.031925,0.009405,0.762061,-0.007699,0.647391),
            5: (-36.098462,0.515858,6.967956,-0.001447,-0.117292,0.001185,0.993096),
        },
        "office": {
            1: (0.000000,0.000000,0.000000,-0.000000,0.000000,0.000000,1.000000),
            2: (-3.444198,-0.008900,0.314562,-0.005836,0.120498,-0.000376,0.992696),
            3: (-3.722319,-0.027628,-0.274054,-0.004965,0.999762,-0.020924,0.003740),
            4: (-3.581410,-0.018210,-0.046041,-0.004849,0.989799,-0.019389,0.141061),
            5: (-3.535166,0.001734,0.857186,0.003445,0.033682,0.000188,0.999427),
            6: (0.012155,-0.000593,-0.099704,-0.004612,0.024890,-0.000660,0.999679),
            7: (-3.510364,-0.000136,0.681082,-0.008888,0.198771,-0.000162,0.980006),
        },
    },
    "d400_imu": {
        "home": {
            1: (0.000000,-0.000000,0.000000,-0.000000,0.000000,0.000000,1.000000),
            2: (2.019698,0.021305,4.575453,-0.006943,-0.562634,0.005684,0.826657),
            3: (-0.818771,0.083748,7.289096,-0.008322,-0.674436,0.006814,0.738255),
            4: (-2.492207,0.073852,4.265824,0.005526,0.447852,-0.004525,0.894079),
            5: (0.876524,0.002188,1.287055,0.009828,0.796461,-0.008047,0.604556),
        },
        "market": {
            1: (0.000000,-0.000000,0.000000,0.000000,-0.000000,-0.000000,1.000000),
            2: (1.178006,-2.212088,15.514719,-0.000561,0.059570,0.008536,0.998187),
            3: (0.936309,-2.571069,18.004021,0.000214,-0.022766,-0.003262,0.999735),
        },
        "cafe": {
            1: (0.000000,-0.000000,0.000000,-0.000000,0.000000,0.000000,1.000000),
            2: (-0.579084,0.008727,0.156544,-0.002065,-0.167310,0.001690,0.985901),
        },
        "corridor": {
            1: (0.000000,-0.000000,0.000000,-0.000000,0.000000,0.000000,1.000000),
            2: (-36.231318,0.539714,9.166994,-0.001044,-0.084696,0.000856,0.996406),
            3: (-36.609250,0.547237,9.450094,-0.001060,-0.085936,0.000868,0.996300),
            4: (-47.121035,0.794048,21.040113,0.009405,0.762061,-0.007699,0.647391),
            5: (-36.095869,0.515842,6.969581,-0.001447,-0.117292,0.001185,0.993096),
        },
        "office": {
            1: (0.000000,-0.000000,0.000000,-0.000000,0.000000,0.000000,1.000000),
            2: (-3.447164,-0.009047,0.313643,-0.005836,0.120498,-0.000376,0.992696),
            3: (-3.733398,-0.027188,-0.250411,-0.004965,0.999762,-0.020924,0.003740),
            4: (-3.595489,-0.017855,-0.024375,-0.004849,0.989799,-0.019389,0.141061),
            5: (-3.535968,0.001818,0.856806,0.003445,0.033682,0.000188,0.999427),
            6: (0.011558,-0.000710,-0.099917,-0.004612,0.024890,-0.000660,0.999679),
            7: (-3.515357,-0.000360,0.679951,-0.008888,0.198771,-0.000162,0.980006),
        },
    },
    "t265_fisheye1": {
        "home": {
            1: (0.000000,0.000000,0.000000,-0.000000,0.000000,-0.000000,1.000000),
            2: (2.053283,0.018268,4.515752,-0.009045,-0.562597,0.006389,0.826657),
            3: (-0.757234,0.094409,7.241848,-0.010842,-0.674391,0.007658,0.738255),
            4: (-2.429192,0.087896,4.301167,0.007200,0.447822,-0.005085,0.894079),
            5: (0.949122,-0.000684,1.283464,0.012804,0.796409,-0.009044,0.604556),
        },
        "market": {
            1: (-0.000000,0.000000,0.000000,-0.000000,0.000000,-0.000000,1.000000),
            2: (-0.381144,5.625077,14.669697,-0.000916,0.056177,-0.021565,0.998187),
            3: (-0.960520,6.503088,16.981551,0.000350,-0.021470,0.008242,0.999735),
        },
        "cafe": {
            1: (0.000000,0.000000,0.000000,-0.000000,0.000000,-0.000000,1.000000),
            2: (-0.583466,0.011070,0.148770,-0.002690,-0.167299,0.001900,0.985901),
        },
        "corridor": {
            1: (0.000000,0.000000,0.000000,-0.000000,0.000000,-0.000000,1.000000),
            2: (-36.163584,0.688469,9.427901,-0.001361,-0.084691,0.000962,0.996406),
            3: (-36.539423,0.697756,9.713686,-0.001381,-0.085930,0.000976,0.996300),
            4: (-46.900735,0.996956,21.392776,0.012252,0.762010,-0.008653,0.647391),
            5: (-36.045484,0.661581,7.227184,-0.001886,-0.117284,0.001332,0.993096),
        },
        "office": {
            1: (0.000000,0.000000,0.000000,-0.000000,0.000000,-0.000000,1.000000),
            2: (-3.438522,0.004781,0.345892,-0.005380,0.120518,-0.000473,0.992696),
            3: (-3.670629,-0.014126,-0.268092,-0.001309,0.999749,-0.022047,0.003740),
            4: (-3.526115,-0.004800,-0.033364,-0.001220,0.989788,-0.020501,0.141061),
            5: (-3.527940,0.016120,0.885206,0.003575,0.033668,0.000124,0.999427),
            6: (0.011991,-0.000619,-0.098661,-0.004522,0.024907,-0.000655,0.999679),
            7: (-3.499082,0.014310,0.716282,-0.008131,0.198803,-0.000327,0.980006),
        },
    },
    "t265_fisheye2": {
        "home": {
            1: (0.000000,0.000000,0.000000,-0.000000,-0.000000,-0.000000,1.000000),
            2: (1.981370,0.030678,4.589291,-0.010840,-0.562538,0.008440,0.826657),
            3: (-0.865461,0.126201,7.299640,-0.012994,-0.674320,0.010118,0.738255),
            4: (-2.484349,0.111377,4.232548,0.008628,0.447776,-0.006718,0.894079),
            5: (0.859061,0.001877,1.228348,0.015345,0.796325,-0.011948,0.604556),
        },
        "market": {
            1: (0.000000,0.000000,0.000000,0.000000,0.000000,-0.000000,1.000000),
            2: (-0.472924,5.676977,14.638771,-0.000760,0.056097,-0.021779,0.998187),
            3: (-1.066326,6.567627,16.953535,0.000290,-0.021439,0.008324,0.999735),
        },
        "cafe": {
            1: (0.000000,0.000000,0.000000,-0.000000,-0.000000,-0.000000,1.000000),
            2: (-0.588044,0.013818,0.165764,-0.003223,-0.167281,0.002510,0.985901),
        },
        "corridor": {
            1: (0.000000,0.000000,0.000000,-0.000000,-0.000000,-0.000000,1.000000),
            2: (-36.226805,0.835882,9.184199,-0.001631,-0.084682,0.001271,0.996406),
            3: (-36.604617,0.847412,9.467483,-0.001656,-0.085921,0.001289,0.996300),
            4: (-47.119124,1.223038,20.998848,0.014683,0.761931,-0.011432,0.647391),
            5: (-36.094352,0.800388,6.988578,-0.002260,-0.117272,0.001760,0.993096),
        },
        "office": {
            1: (0.000000,0.000000,0.000000,-0.000000,-0.000000,-0.000000,1.000000),
            2: (-3.442659,0.016500,0.306628,-0.005001,0.120532,-0.000964,0.992696),
            3: (-3.796658,-0.003576,-0.294144,0.001956,0.999658,-0.025813,0.003740),
            4: (-3.651193,0.005770,-0.075837,0.002003,0.989702,-0.024230,0.141061),
            5: (-3.534078,0.030372,0.856267,0.003678,0.033657,0.000022,0.999427),
            6: (0.012596,-0.001142,-0.101759,-0.004440,0.024918,-0.000780,0.999679),
            7: (-3.508952,0.027483,0.666926,-0.007510,0.198825,-0.001131,0.980006),
        },
    },
    "t265_imu": {
        "home": {
            1: (0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,1.000000),
            2: (-1.996198,0.003803,4.548154,0.011874,0.562561,0.004741,0.826657),
            3: (0.846974,-0.078926,7.243782,0.014233,0.674347,0.005683,0.738255),
            4: (2.480536,-0.088306,4.265572,-0.009452,-0.447793,-0.003774,0.894079),
            5: (-0.921339,0.008630,1.283449,-0.016809,-0.796357,-0.006712,0.604556),
        },
        "market": {
            1: (0.000000,-0.000000,0.000000,0.000000,0.000000,-0.000000,1.000000),
            2: (0.516480,-5.586881,14.678775,0.000401,-0.056239,-0.021419,0.998187),
            3: (1.117126,-6.462140,16.988129,-0.000153,0.021493,0.008186,0.999735),
        },
        "cafe": {
            1: (0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,1.000000),
            2: (0.585658,-0.013591,0.145843,0.003531,0.167288,0.001410,0.985901),
        },
        "corridor": {
            1: (0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,1.000000),
            2: (36.262398,-0.841508,9.029932,0.001786,0.084685,0.000714,0.996406),
            3: (36.641342,-0.851879,9.311582,0.001814,0.085925,0.000724,0.996300),
            4: (47.142230,-1.170879,20.863299,-0.016084,-0.761961,-0.006422,0.647391),
            5: (36.120152,-0.819977,6.831279,0.002475,0.117277,0.000988,0.993096),
        },
        "office": {
            1: (0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,1.000000),
            2: (3.442369,-0.020799,0.305186,0.004783,-0.120544,-0.000201,0.992696),
            3: (3.688849,-0.004784,-0.309137,0.003839,0.999799,0.019311,-0.003740),
            4: (3.546448,-0.012713,-0.075700,0.003862,0.989833,0.017793,-0.141061),
            5: (3.537446,-0.031149,0.845348,-0.003738,-0.033650,0.000256,0.999427),
            6: (-0.013075,0.000427,-0.099056,0.004392,-0.024931,-0.000637,0.999679),
            7: (3.507507,-0.029618,0.673291,0.007152,-0.198841,0.000130,0.980006),
        },
    },
    "base_link": {
        "home": {
            1: (0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000),
            2: (4.653439,-2.246949,0.000000,0.000000,0.000000,0.562706,0.826657),
            3: (7.410862,0.568846,0.000000,0.000000,0.000000,0.674521,0.738255),
            4: (4.399963,2.653627,0.000000,0.000000,0.000000,-0.447909,0.894079),
            5: (1.633000,-0.754733,0.000000,0.000000,0.000000,-0.796563,0.604556),
        },
        "market": {
            1: (0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000),
            2: (15.713963,-0.293285,0.000000,0.000000,0.000000,-0.060181,0.998187),
            3: (18.211526,0.017624,0.000000,0.000000,0.000000,0.023000,0.999735),
        },
        "cafe": {
            1: (0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000),
            2: (0.142683,0.504664,0.000000,0.000000,0.000000,0.167331,0.985901),
        },
        "corridor": {
            1: (0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000),
            2: (9.014827,36.233859,-0.000016,0.000000,-0.000001,0.084707,0.996406),
            3: (9.296355,36.612415,-0.000015,0.000000,0.000000,0.085947,0.996300),
            4: (21.175313,47.339619,-0.000017,-0.000000,-0.000001,-0.762158,0.647391),
            5: (6.816094,36.074646,-0.000017,0.000000,0.000000,0.117307,0.993096),
        },
        "office": {
            1: (0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000),
            2: (0.309635,3.500538,0.058010,0.000811,0.007326,-0.120414,0.992696),
            3: (0.145258,3.621898,0.064342,0.010893,-0.017257,0.999785,-0.003740),
            4: (0.381370,3.543278,0.057061,0.009458,-0.017024,0.989809,-0.141061),
            5: (0.853251,3.554253,0.049278,0.000541,-0.003027,-0.033718,0.999427),
            6: (-0.105108,-0.001915,0.001637,-0.000429,0.004917,-0.024837,0.999679),
            7: (0.690678,3.603368,0.055720,0.001800,0.011347,-0.198638,0.980006),
        },
    },
}


if __name__ == "__main__":
    main()
    sys.exit(0)

#!/usr/bin/env python2

# Copyright (C) <2019> Intel Corporation
# SPDX-License-Identifier: MIT
# Author: Xuesong Shi

"""
1. convert gx odom to standard types and add tf
2. convert mocap stamp to unix timestamp and merge the messages from tf
3. remove mocap outliers with a median filter
4. remove repeated odom messages
"""
import rosbag
import sys
import numpy as np
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseStamped, TransformStamped
from rospy.rostime import Time, Duration
from tf2_msgs.msg import TFMessage
import argparse

def main():
    topic_mocap = '/vrpn_client_node/RigidBody1/pose'
    frame_mocap = 'RigidBody1'
    odom_frame_id = 'base_odom'
    odom_child_frame_id = 'base_link'

    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--sync-mocap-file", type=str, help="sync file for mocap", default="")
    args, left = parser.parse_known_args()

    if len(left) != 1:
        print ("Usage: " + sys.argv[0] + " odom_and_mocap.bag [-s sync_time.txt]")
        return
    in_file = left[0]
    sync_file = args.sync_mocap_file
    out_file = 'preprocessed.bag'

    ##the time diff between mocap client and server
    if sync_file != '':
        mocap_offset = getMin_diff_ser_clit(sync_file)
        print ('mocap time offset: %f' % mocap_offset)
        mocap_offset = Duration.from_sec(mocap_offset)
    else:
        mocap_offset = None

    with rosbag.Bag(in_file) as inbag:
        print ("-------------------- Input: %s ------------------------------" % in_file)
        print (inbag)
        outbag = rosbag.Bag(out_file, 'w')
        writer_mocap = TopicWriter(outbag, topic_mocap)
        writer_mocap.set_median_filter(1.0, 0.5)
        gx_poses = {}
        gx_stamp_offset = None
        for topic,msg,t in inbag.read_messages():
            if topic=="/gx/RosPose2d":
                if msg.timestamp in gx_poses:
                    old = gx_poses[msg.timestamp]
                    if msg == old:
                        continue
                    else:
                        print old
                        print msg
                        print ('different odom message with identical stamp!')
                        exit()
                if gx_stamp_offset is None:
                    """
                    Use bag time if stamp is abnormal
                    """
                    stamp = gx_msg_stamp(msg)
                    if abs(stamp.to_sec() - t.to_sec()) > 1000:
                        gx_stamp_offset = t.to_sec() - stamp.to_sec()
                        print ('odom time offset: %f' % gx_stamp_offset)
                    else:
                        gx_stamp_offset = 0
                gxPose2d_to_odom(msg,t,outbag,odom_frame_id,odom_child_frame_id,gx_stamp_offset)
                gxPose2d_to_tf(msg,t,outbag,odom_frame_id,odom_child_frame_id,gx_stamp_offset)
                gx_poses[msg.timestamp] = msg
            elif topic == topic_mocap:
                if mocap_offset is None:
                    print ('\nWarning: found mocap topic but sync file is not specified (perhaps run again with [-s sync_time.txt]?)\n')
                    mocap_offset = Duration.from_sec(0)
                msg.header.stamp += mocap_offset
                writer_mocap.write_sliding(msg, t)
            elif topic=="/gx/RosBaseImu":
                pass
            elif topic=="/gx/RosBaseTicks":
                pass
            elif topic == '/tf' and msg.transforms[0].child_frame_id == frame_mocap:
                # merge mocap tf into its topic
                msg.transforms[0].header.stamp += mocap_offset
                writer_mocap.write_sliding(tf_to_pose(msg), t)
            else:
                outbag.write(topic,msg,t)

        outbag.close()
        outbag = rosbag.Bag(out_file)
        print ("------------------- Output: %s ------------------------------" % out_file)
        print (outbag)


class TopicWriter(object):

    __outbag = None
    __topic = None
    __filter = False
    __filter_max_interval = None
    __filter_diff_tol = None
    __last_msg = None
    __curr_msg = None
    __curr_t = None
    __sliding_window_msg=[]
    __sliding_window_t=[]
    __sliding_window_size=None
    __processed = set()
    __seq = 0
    __number_of_outlier=0
    __sliding_window_euler=[]
    __filter_diff_euler=None

    def __init__(self, outbag, topic):
        self.__outbag = outbag
        self.__topic = topic
        self.__sliding_window_msg=[]
        self.__sliding_window_t=[]
        self.__sliding_window_size=3
        self.__number_of_outlier=0
        self.__sliding_window_euler=[]
        self.__filter_diff_euler=120

    def set_median_filter(self, max_interval, tol):
        self.__filter = True
        self.__filter_max_interval = max_interval
        self.__filter_diff_tol = tol
    '''
    def write(self, msg, t):
        # filter out repeated messages
        key = msg.header.stamp.to_nsec()
        if key in self.__processed:
            return
        self.__processed.add(key)
        if not self.__filter:
            self.__write(msg, t)
            return
        elif not self.__curr_msg: # 1st msg
            pass
        elif not self.__last_msg: # 2nd msg
            if self.__check(msg, self.__curr_msg, msg):
                self.__write(self.__curr_msg, self.__curr_t)
        else:
            if self.__check(self.__last_msg, self.__curr_msg, msg):
                self.__write(self.__curr_msg, self.__curr_t)
        self.__last_msg = self.__curr_msg
        self.__curr_msg = msg
        self.__curr_t = t
    '''
    def write_sliding(self, msg, t):
        # filter out repeated messages
        key = msg.header.stamp.to_nsec()
        if key in self.__processed:
            return
        self.__processed.add(key)
        if len(self.__sliding_window_msg) < self.__sliding_window_size:
            self.__sliding_window_msg.append(msg)
            self.__sliding_window_t.append(t)
            self.__sliding_window_euler.append(msg)
        else:
            if self.__check_both_X_and_euler(msg,self.__sliding_window_msg, self.__sliding_window_euler):
                self.__write(self.__sliding_window_msg[0],self.__sliding_window_t[0])
                self.__sliding_window_msg = self.__sliding_window_msg[1:] + [msg]
                self.__sliding_window_t = self.__sliding_window_t[1:] + [t]
                self.__sliding_window_euler = self.__sliding_window_euler[1:] + [msg]
    '''
    def __check(self, msg1, msg2, msg3):
        if msg3.header.stamp.to_sec() - msg2.header.stamp.to_sec() > self.__filter_max_interval or \
            msg2.header.stamp.to_sec() - msg1.header.stamp.to_sec() > self.__filter_max_interval:
            return True
        if abs(msg2.pose.position.x - (msg1.pose.position.x + msg3.pose.position.x) * 0.5) > self.__filter_diff_tol:
            print ("[outlier #%d] position.x: " % msg2.header.seq, msg1.pose.position.x, msg2.pose.position.x, msg3.pose.position.x)
            return False
        if abs(msg2.pose.position.y - (msg1.pose.position.y + msg3.pose.position.y) * 0.5) > self.__filter_diff_tol:
            print ("[outlier #%d] position.y: " % msg2.header.seq, msg1.pose.position.x, msg2.pose.position.x, msg3.pose.position.x)
            return False
        if abs(msg2.pose.position.z - (msg1.pose.position.z + msg3.pose.position.z) * 0.5) > self.__filter_diff_tol:
            print ("[outlier #%d] position.z: " % msg2.header.seq, msg1.pose.position.x, msg2.pose.position.x, msg3.pose.position.x)
            return False
        if abs(msg2.pose.orientation.x - (msg1.pose.orientation.x + msg3.pose.orientation.x) * 0.5) > self.__filter_diff_tol:
            print ("orientation.x: ", msg1.pose.orientation.x, msg2.pose.orientation.x, msg3.pose.orientation.x)
            print ("orientation.y: ", msg1.pose.orientation.y, msg2.pose.orientation.y, msg3.pose.orientation.y)
            print ("orientation.z: ", msg1.pose.orientation.z, msg2.pose.orientation.z, msg3.pose.orientation.z)
            print ("orientation.w: ", msg1.pose.orientation.w, msg2.pose.orientation.w, msg3.pose.orientation.w)
            return False
        return True
    '''
    def __write(self, msg, t):
        msg.header.seq = self.__seq
        self.__seq += 1
        self.__outbag.write(self.__topic, msg, t)

    def __check__sliding(self, msg, sliding):
        if abs(msg.pose.orientation.x - self.calculate_mean(sliding)) > self.__filter_diff_tol:

            print ("------------------------------------------------------------------------------")
            print ("orientation---outlier: ", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            for i in range(len(sliding)):
                print ("orientation----inlier: ", sliding[i].pose.orientation.x, sliding[i].pose.orientation.y, sliding[i].pose.orientation.z, sliding[i].pose.orientation.w)
            return False
        return True

    def __check__sliding_euler(self, msg, sliding_euler):
        if abs(self.quaternion_to_euler(msg)- self.calculate_mean_euler(sliding_euler)) > self.__filter_diff_euler:
            print self.__number_of_outlier
            print ("------------------------------------------------------------------------------")
            print ("euler---outlier X = : ", self.quaternion_to_euler(msg))
            for i in range(len(sliding_euler)):
                print ("euler---inlier X = : ",self.quaternion_to_euler(sliding_euler[i]))
            return False
        return True

    def __check_both_X_and_euler(self, msg, sliding, sliding_euler):
        if self.__check__sliding(msg, sliding) and self.__check__sliding_euler(msg, sliding_euler):
            return True
        else:
            self.__number_of_outlier=self.__number_of_outlier+1
            print ("Number of outliers: ", self.__number_of_outlier)
            return False

    def calculate_mean(self,sliding):
        m=0
        for i in range(len(sliding)):
            m=m+sliding[i].pose.orientation.x
            return m/len(sliding)

    def calculate_mean_euler(self,sliding):
        m=0
        for i in range(len(sliding)):
            m=m+self.quaternion_to_euler(sliding[i])
            return m/len(sliding)

    def quaternion_to_euler(self, msg):
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))
        '''
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))
        '''
        return X

def gx_msg_stamp(msg):
    s = str(msg)
    if s.startswith('timestamp: '):
        stamp = s.split('\n')[0].split(':')[1].strip()
        stamp = stamp[:10] + '.' + stamp[10:]
        return Time.from_sec(float(stamp))
    else:
        print (msg)
        exit('Unknown gx type!')

def gxPose2d_to_odom(msg,t,outbag,frame_id,child_frame_id,offset):
                '''
                turn /gx/RosPose2d topic to /odom topic
                change the type of gx_sensor/RosPose2d to nav_msgs/Odometry
                /gx/RosPose2d topic message example:
                                timestamp: 1562430246571000
                                x: 2.03333210945
                                y: -0.198542118073
                                theta: 0.551429688931
                                linearVelocity: 0.0
                                angularVelociy: -0.00757710635662
                the topic is in base_link coordinate
                '''
                str_gx=[x.split(":") for x in str(msg).split("\n")]
                unit_odom=Odometry()
                unit_odom.pose.pose.position=Point(float(str_gx[1][1]),float(str_gx[2][1]),0)
                unit_odom.pose.pose.orientation=Quaternion(*tf.transformations.quaternion_from_euler(0,0,float(str_gx[3][1])))
                unit_odom.header.stamp=Time.from_sec(gx_msg_stamp(msg).to_sec() + offset)
                unit_odom.header.frame_id=frame_id
                unit_odom.child_frame_id=child_frame_id
                unit_odom.twist.twist.angular=Point(0,0,float(str_gx[5][1]))
                unit_odom.twist.twist.linear=Point(float(str_gx[4][1])*math.cos(float(str_gx[3][1])),float(str_gx[4][1])*math.sin(float(str_gx[3][1])),0)
                outbag.write('/odom',unit_odom,t)

def tf_to_pose(tf_msg):
    pose_msg = PoseStamped()
    pose_msg.header = tf_msg.transforms[0].header
    pose_msg.pose.position = tf_msg.transforms[0].transform.translation
    pose_msg.pose.orientation = tf_msg.transforms[0].transform.rotation
    return pose_msg

def gxPose2d_to_tf(msg,t,outbag,frame_id,child_frame_id,offset):
                str_gx=[x.split(":") for x in str(msg).split("\n")]
                br = TransformStamped()
                br.transform.translation=Point(float(str_gx[1][1]),float(str_gx[2][1]),0)
                br.transform.rotation=Quaternion(*tf.transformations.quaternion_from_euler(0,0,float(str_gx[3][1])))
                br.header.stamp=Time.from_sec(gx_msg_stamp(msg).to_sec() + offset)
                br.header.frame_id=frame_id
                br.child_frame_id=child_frame_id
                tfm=TFMessage([br])
                outbag.write('/tf',tfm,t)


def base_laser(msg,t,outbag,laser_frame_id):
                new_msg=msg
                new_msg.header.frame_id=laser_frame_id
                print (msg.header.frame_id, laser_frame_id)
                outbag.write('/scan',new_msg,t)


def getMin_diff_ser_clit(sync_time):
    with open(sync_time,'r') as f:
        lines = f.read().splitlines()
        last_line = lines[-1]
        return float(last_line.split("  ")[4][:10]+"."+last_line.split("  ")[4][10:])


if __name__ == '__main__':
    main()

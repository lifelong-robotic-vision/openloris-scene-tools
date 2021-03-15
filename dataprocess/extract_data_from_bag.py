#!/usr/bin/python
import rosbag
import rospy
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import os
import sys
import numpy as np
import csv
import argparse

def extract_d400_data(inbag, outfolder):
    print ("extracting D400 RGBD and IMU data from " + inbag + " to " + outfolder)
    cv_bridge = CvBridge()
    if not os.path.isdir(outfolder + 'color/'):
        os.mkdir(outfolder + 'color/')
    if not os.path.isdir(outfolder + 'depth/'):
        os.mkdir(outfolder + 'depth/')
    if not os.path.isdir(outfolder + 'aligned_depth/'):
        os.mkdir(outfolder + 'aligned_depth/')
    timestr_1 = []
    timestr_2 = []
    timestr_3 = []
    with rosbag.Bag(inbag, 'r') as bag:
        N_acc = bag.get_message_count('/d400/accel/sample')
        acc = np.zeros((4, N_acc))
        acc_cnt = 0
        N_gyr = bag.get_message_count('/d400/gyro/sample')
        gyr = np.zeros((4, N_gyr))
        gyr_cnt = 0
        for topic, msg, t in bag.read_messages():
            if topic == "/d400/color/image_raw":
                try:
                    cv_image = cv_bridge.imgmsg_to_cv2(msg,"bgr8")
                except CvBridgeError as e:
                    print(e)
                timestr = "%.6f" % msg.header.stamp.to_sec()
                timestr_1.append(timestr)
                image_name = timestr+ ".png"
                cv2.imwrite(outfolder+'color/'+image_name, cv_image)
            if topic == "/d400/depth/image_raw":
                try:
                    msg.encoding = "mono16"
                    cv_image = cv_bridge.imgmsg_to_cv2(msg,"mono16")
                except CvBridgeError as e:
                    print(e)
                timestr = "%.6f" % msg.header.stamp.to_sec()
                timestr_2.append(timestr)
                image_name = timestr+ ".png"
                cv2.imwrite(outfolder+'depth/'+image_name, cv_image)
            if topic == "/d400/aligned_depth_to_color/image_raw":
                try:
                    msg.encoding = "mono16"
                    cv_image = cv_bridge.imgmsg_to_cv2(msg,"mono16")
                except CvBridgeError as e:
                    print(e)
                timestr = "%.6f" % msg.header.stamp.to_sec()
                timestr_3.append(timestr)
                image_name = timestr+ ".png"
                cv2.imwrite(outfolder+'aligned_depth/'+image_name, cv_image)
            if topic == "/d400/accel/sample":
                acc[1, acc_cnt] = msg.linear_acceleration.x
                acc[2, acc_cnt] = msg.linear_acceleration.y
                acc[3, acc_cnt] = msg.linear_acceleration.z
                acc[0, acc_cnt] = msg.header.stamp.secs + msg.header.stamp.nsecs * pow(10, -9)
                acc_cnt = acc_cnt + 1
            if topic == "/d400/gyro/sample":
                gyr[1, gyr_cnt] = msg.angular_velocity.x
                gyr[2, gyr_cnt] = msg.angular_velocity.y
                gyr[3, gyr_cnt] = msg.angular_velocity.z
                gyr[0, gyr_cnt] = msg.header.stamp.secs + msg.header.stamp.nsecs * pow(10, -9)
                gyr_cnt = gyr_cnt + 1
    timestr_1.sort()
    timestr_2.sort()
    f1 = open(outfolder + "color.txt", "w")
    for timestr in timestr_1:
        f1.writelines(timestr + ' color/' + timestr + ".png\n")
    f1.close()
    f2 = open(outfolder + "depth.txt", "w")
    for timestr in timestr_2:
        f2.writelines(timestr + ' depth/' + timestr + ".png\n")
    f2.close()
    f2 = open(outfolder + "aligned_depth.txt", "w")
    for timestr in timestr_3:
        f2.writelines(timestr + ' aligned_depth/' + timestr + ".png\n")
    f2.close()
    acc = acc[:, acc[0].argsort()]
    gyr = gyr[:, gyr[0].argsort()]

    fname = 'd400_accelerometer'
    f3 = open(outfolder + fname + '.txt', 'wt')
    try:
        f3.writelines('#Time Ax Ay Az\n')
        for i in range(acc[1, :].size):
            f3.writelines("%.8f" % acc[0, i]+" "+str(acc[1, i])+" "+str(acc[2, i])+" "+str(acc[3, i])+ "\n")
    finally:
        f3.close()

    fname = 'd400_gyroscope'
    f4 = open(outfolder + fname + '.txt', 'wt')
    try:
        f4.writelines('#Time Gx Gy Gz\n')
        for i in range(gyr[1, :].size):
            f4.writelines("%.8f" % gyr[0, i]+" "+str(gyr[1, i])+" "+str(gyr[2, i])+" "+str(gyr[3, i])+ "\n")
    finally:
        f4.close()
    print ("done.")

def extract_t265_data(inbag, outfolder):
    print ("extracting T265 fisheye and IMU data from " + inbag + " to " + outfolder)
    cv_bridge = CvBridge()
    if not os.path.isdir(outfolder + 'fisheye1/'):
        os.mkdir(outfolder + 'fisheye1/')
    if not os.path.isdir(outfolder + 'fisheye2/'):
        os.mkdir(outfolder + 'fisheye2/')
    timestr_1 = []
    timestr_2 = []
    with rosbag.Bag(inbag, 'r') as bag:
        N_acc = bag.get_message_count('/t265/accel/sample')
        acc = np.zeros((4, N_acc))
        acc_cnt = 0
        N_gyr = bag.get_message_count('/t265/gyro/sample')
        gyr = np.zeros((4, N_gyr))
        gyr_cnt = 0
        for topic, msg, t in bag.read_messages():
            if topic == "/t265/fisheye1/image_raw":
                try:
                    msg.encoding = "mono8"
                    cv_image = cv_bridge.imgmsg_to_cv2(msg,"mono8")
                except CvBridgeError as e:
                    print(e)
                timestr = "%.6f" % msg.header.stamp.to_sec()
                timestr_1.append(timestr)
                image_name = timestr+ ".png"
                cv2.imwrite(outfolder+'fisheye1/'+image_name, cv_image)
            if topic == "/t265/fisheye2/image_raw":
                try:
                    msg.encoding = "mono8"
                    cv_image = cv_bridge.imgmsg_to_cv2(msg, "mono8")
                except CvBridgeError as e:
                    print(e)
                timestr = "%.6f" % msg.header.stamp.to_sec()
                timestr_2.append(timestr)
                image_name = timestr+ ".png"
                cv2.imwrite(outfolder+'fisheye2/'+image_name, cv_image)
            if topic == "/t265/accel/sample":
                acc[1, acc_cnt] = msg.linear_acceleration.x
                acc[2, acc_cnt] = msg.linear_acceleration.y
                acc[3, acc_cnt] = msg.linear_acceleration.z
                acc[0, acc_cnt] = msg.header.stamp.secs + msg.header.stamp.nsecs * pow(10, -9)
                acc_cnt = acc_cnt + 1
            if topic == "/t265/gyro/sample":
                gyr[1, gyr_cnt] = msg.angular_velocity.x
                gyr[2, gyr_cnt] = msg.angular_velocity.y
                gyr[3, gyr_cnt] = msg.angular_velocity.z
                gyr[0, gyr_cnt] = msg.header.stamp.secs + msg.header.stamp.nsecs * pow(10, -9)
                gyr_cnt = gyr_cnt + 1
    timestr_1.sort()
    timestr_2.sort()
    f1 = open(outfolder + "fisheye1.txt", "w")
    for timestr in timestr_1:
        f1.writelines(timestr + ' fisheye1/' + timestr + ".png\n")
    f1.close()
    f2 = open(outfolder + "fisheye2.txt", "w")
    for timestr in timestr_2:
        f2.writelines(timestr + ' fisheye2/' + timestr + ".png\n")
    f2.close()
    acc = acc[:, acc[0].argsort()]
    gyr = gyr[:, gyr[0].argsort()]
    fname = 't265_accelerometer'
    f3 = open(outfolder + fname + '.txt', 'wt')
    try:
        f3.writelines('#Time Ax Ay Az\n')
        for i in range(acc[1, :].size):
            f3.writelines("%.8f" % acc[0, i] + " " + str(acc[1, i]) + " " + str(acc[2, i]) + " " + str(acc[3, i])+ "\n")
    finally:
        f3.close()

    fname = 't265_gyroscope'
    f4 = open(outfolder + fname + '.txt', 'wt')
    try:
        f4.writelines('#Time Gx Gy Gz\n')
        for i in range(gyr[1, :].size):
            f4.writelines("%.8f" % gyr[0, i] + " " + str(gyr[1, i]) + " " + str(gyr[2, i]) + " " + str(gyr[3, i])+ "\n")
    finally:
        f4.close()
    print ("done.")


def extract_odom_gt(inbag, outfolder):
    print ("extracting odom and ground truth from " + inbag + " to " + outfolder)

    with rosbag.Bag(inbag, 'r') as bag:
        N_odom = bag.get_message_count('/odom')
        odom = np.zeros((14, N_odom))
        odom_cnt = 0
        N_pose = bag.get_message_count('/gt')
        pose = np.zeros((8, N_pose))
        pose_cnt = 0
        for topic, msg, t in bag.read_messages():
            if topic == "/odom":
                odom[1, odom_cnt] = msg.pose.pose.position.x
                odom[2, odom_cnt] = msg.pose.pose.position.y
                odom[3, odom_cnt] = msg.pose.pose.position.z
                odom[4, odom_cnt] = msg.pose.pose.orientation.x
                odom[5, odom_cnt] = msg.pose.pose.orientation.y
                odom[6, odom_cnt] = msg.pose.pose.orientation.z
                odom[7, odom_cnt] = msg.pose.pose.orientation.w
                odom[8, odom_cnt] = msg.twist.twist.linear.x
                odom[9, odom_cnt] = msg.twist.twist.linear.y
                odom[10, odom_cnt] = msg.twist.twist.linear.z
                odom[11, odom_cnt] = msg.twist.twist.angular.x
                odom[12, odom_cnt] = msg.twist.twist.angular.y
                odom[13, odom_cnt] = msg.twist.twist.angular.z
                odom[0, odom_cnt] = msg.header.stamp.secs + msg.header.stamp.nsecs * pow(10, -9);
                odom_cnt = odom_cnt + 1
            if topic == "/gt":
                pose[1, pose_cnt] = msg.transforms[0].transform.translation.x
                pose[2, pose_cnt] = msg.transforms[0].transform.translation.y
                pose[3, pose_cnt] = msg.transforms[0].transform.translation.z
                pose[4, pose_cnt] = msg.transforms[0].transform.rotation.x
                pose[5, pose_cnt] = msg.transforms[0].transform.rotation.y
                pose[6, pose_cnt] = msg.transforms[0].transform.rotation.z
                pose[7, pose_cnt] = msg.transforms[0].transform.rotation.w
                pose[0, pose_cnt] = msg.transforms[0].header.stamp.secs + msg.transforms[0].header.stamp.nsecs * pow(10, -9);
                pose_cnt = pose_cnt + 1
            if topic == "/tf_static":
                pass
    odom = odom[:, odom[0].argsort()]
    pose = pose[:, pose[0].argsort()]
    fname = 'odom'
    f1 = open(outfolder + fname + '.txt', 'wt')
    try:
        f1.writelines('#Time pose.position.x y z pose.orientation.x y z w twist.linear.x y z twist.angular.x y z\n')
        for i in range(odom[1, :].size):
            f1.writelines(
                "%.8f" % odom[0, i] +" "+ str(odom[1, i])+" "+ str(odom[2, i])+" "+ str(odom[3, i])+" "+ str(odom[4, i])+" "+ str(odom[5, i])+" "+ str(odom[6, i])
                +" "+ str(odom[7, i])+" "+ str(odom[8, i])+" "+ str(odom[9, i])+" "+ str(odom[10, i])+" "+ str(odom[11, i])+" "+ str(odom[12, i])+" "+ str(odom[13, i])+ "\n")
    finally:
        f1.close()

    fname = 'groundtruth'
    f2 = open(outfolder + fname + '.txt', 'w')
    f2.writelines('#Time px py pz qx qy qz qw\n')
    for i in range(pose[1, :].size):
        f2.writelines(
            "%.8f" % pose[0, i] + " " + str(pose[1, i]) + " " + str(pose[2, i]) + " " + str(pose[3, i]) + " " + str(
                pose[4, i]) + " " + str(pose[5, i]) + " " + str(pose[6, i]) + " " + str(pose[7, i]) + "\n")
    f2.close()
    print ("done.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--inbag', type=str, default='office1-1.bag', \
        help='input bag file')
    parser.add_argument('-o', '--outfolder', type=str, default='', \
        help='output folder')
    args = parser.parse_args()

    inbag = args.inbag
    outfolder = args.outfolder + '/' if args.outfolder != '' else args.inbag.rsplit('.', 1)[0] + '/'
    if not os.path.exists(outfolder):
        os.mkdir(outfolder)

    extract_d400_data(inbag, outfolder)
    extract_t265_data(inbag, outfolder)
    extract_odom_gt(inbag, outfolder)

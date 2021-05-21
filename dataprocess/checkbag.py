#!/usr/bin/env python2

# Copyright (C) <2019> Intel Corporation
# SPDX-License-Identifier: MIT
# Author: Xuesong Shi

import rosbag
import sys
import numpy
import math
import argparse
import std_msgs
from rospy.rostime import Time
from std_msgs.msg import Header
from termcolor import colored
import matplotlib.pyplot as plt

def check_time_consistency(headers, times):
    ht0 = headers[0].stamp.to_sec()
    mt0 = times[0].to_sec()
    for h, t in zip(headers, times):
        ht = h.stamp.to_sec() - ht0
        mt = t.to_sec() - mt0
        if abs(ht - mt) > 0.05:
            print ('header stamp: %f / t: %f' % (ht, mt))

def check_times(headers, times):
    mt0 = times[0].to_sec()
    mt1 = times[0].to_sec()
    last_mt = mt0
    last_ht = headers[0].stamp.to_sec()
    for h, t in zip(headers, times):
        ht = h.stamp.to_sec()
        mt = t.to_sec()
        if mt - last_mt > 0.08:
            print ('No message for %.3f sec (%.1f to %.1f). t: %f -> %f. header: %f -> %f (+%f)' % \
                (mt - last_mt, last_mt - mt0, mt - mt0, last_mt, mt, last_ht, ht, ht - last_ht))
        last_mt = mt
        last_ht = ht

def plot_times(topic, headers, times):
    mt0 = times[0].to_sec()
    ht0 = headers[0].stamp.to_sec()
    hts = [h.stamp.to_sec() - ht0 for h in headers]
    mts = [t.to_sec() - mt0 for t in times]
    plt.plot(mts, hts, '.r')
    plt.hold(True)
    tmax = times[-1].to_sec() - mt0
    plt.plot([0,tmax], [0, tmax])
    plt.title(topic)
    plt.show()

def check_headers(headers):
    all_t = set()
    logs = {}
    log_ids = []
    intervals = []
    tmin = float('inf')
    tmin_world = float('inf')
    tmax = -float('inf')
    last_seq = float('nan')
    ndrop = 0
    for h in headers:
        seq = h.seq
        t = h.stamp.to_sec()
        all_t.add(t)
        if t > tmax: tmax = t
        if t < tmin: tmin = t
        if t < tmin_world and t > 1000000000: tmin_world = t
        if math.isnan(last_seq):
            last_seq = seq
            last_t = t
            continue
        delta = (t - last_t) * 1000
        intervals.append(delta)
        if (seq != 0 or last_seq != 0) and seq != last_seq+1:
            logs[t] = (("message drop (" + colored("seq %d->%d", 'red') + " interval %f ms)") % (last_seq, seq, delta))
            ndrop += seq - last_seq - 1
        elif delta < 0:
            logs[t] = (("out-of-order (seq %d->%d " + colored("interval %f ms", 'red') + ")") % (last_seq, seq, delta))
        elif delta > 1e9:
            logs[t] = (("invalid stamp (seq %d->%d " + colored("stamp %f->%f", 'red') + " interval %f ms)") % (last_seq, seq, last_t, t, delta))
            intervals[-1] = float('nan')
        last_seq = seq
        last_t = t

    if not math.isinf(tmin_world): tmin = tmin_world
    if tmax == tmin:
        print ('All messages have the same stamp: %f' % tmin)
        return

    ints = numpy.array(intervals)
    median = numpy.nanmedian(ints)
    ints = numpy.ma.array(ints, mask=(numpy.isnan(ints)))
    print ("%f FPS in %.2f sec. Intervals (ms): median %f, max %f, min %f, std %f" % \
        (1000/median, tmax - tmin, median, numpy.max(ints), numpy.min(ints), numpy.std(ints)))

    with numpy.errstate(invalid='ignore'):
        idx = numpy.where(ints > 1.5 * median)[0]
    for i in idx:
        t = headers[i+1].stamp.to_sec()
        if not logs.has_key(t):
            logs[t] = (("large interval (seq %d->%d " + colored("interval %f ms", 'red') + ")") % (headers[i].seq, headers[i+1].seq, ints[i]))

    with numpy.errstate(invalid='ignore'):
        idx = numpy.where(ints < 0.5 * median)[0]
    for i in idx:
        t = headers[i+1].stamp.to_sec()
        if not logs.has_key(t):
            logs[t] = (("small interval (seq %d->%d " + colored("interval %f ms", 'red') + ")") % (headers[i].seq, headers[i+1].seq, ints[i]))

    if args.verbose:
        for t in sorted(logs.keys()): print ("\t" + "%2.0f%%: " % ((t-tmin) / (tmax-tmin) * 100)  + logs[t])
    if True:
        nlarge = 0
        norder = 0
        ninvalid = 0
        nsmall = 0
        msg = list('X' * 101)
        for t in all_t:
            perc = int((t-tmin) / (tmax-tmin) * 100)
            if perc < 0 or perc > 100: continue
            msg[perc] = '.'
        for t in logs.keys():
            perc = int((t-tmin) / (tmax-tmin) * 100)
            if perc < 0 or perc > 100: print perc
            if 'drop' in logs[t]:
                nlarge += 1
                msg[perc] = 'D'
            elif 'large' in logs[t]:
                nlarge += 1
                msg[perc] = 'L'
            elif 'out-of-order' in logs[t]:
                norder += 1
                msg[perc] = 'O'
            elif 'invalid' in logs[t]:
                ninvalid += 1
                msg[perc] = 'I'
            elif 'small' in logs[t]:
                nsmall += 1
                if msg[perc] == '.': msg[perc] = 's'
        ratio = 100. / len(headers)
        ntotal = ndrop + norder + ninvalid + nsmall
        print ('%d drops (%.1f%%); %d out-of-order (%.1f%%); %d invalid stamp (%.1f%%); %d large intervals (%.1f%%); %d small intervals (%.1f%%); %d total abnormal (%.1f%%)' \
            % (ndrop, ndrop * ratio, norder, norder * ratio, ninvalid, ninvalid * ratio, nlarge, nlarge * ratio, nsmall, nsmall * ratio, ntotal, ntotal * ratio))
        print (''.join(msg))

def print_highlight(s):
    print (colored(s, 'yellow'))

def get_stamp_from_gx_msg(msg):
    str_gx = [x.split(":") for x in str(msg).split("\n")]
    stamp = str_gx[0][1].strip()[:10]+"."+str_gx[0][1].strip()[10:]
    stamp = Time.from_sec(numpy.float128(stamp))
    return stamp

def check_bag(bag):
    topic_headers = {}
    topic_times = {}
    topic_count = {}
    one_msg_topics = []
    for topic, msg, t in bag.read_messages():
        if 'notification' in topic:
            print (topic + ': (t=%f)' % t.to_sec())
            print (msg)
        if hasattr(msg, 'header'):
            if topic not in topic_headers:
                topic_headers[topic] = [msg.header]
            else:
                topic_headers[topic].append(msg.header)
            topic_times.setdefault(topic, []).append(t)
        elif topic.startswith('/gx/'):
            header = Header()
            header.stamp = get_stamp_from_gx_msg(msg)
            header.seq = 0
            topic_headers.setdefault(topic, []).append(header)
            topic_times.setdefault(topic, []).append(t)
        else:
            if topic not in topic_count:
                topic_count[topic] = 1
            else:
                topic_count[topic] += 1

    for topic in topic_headers:
        if len(topic_headers[topic]) is 1:
            one_msg_topics.append(topic)
            continue
        print_highlight (topic + ": %d messages" % len(topic_headers[topic]))
        check_headers(topic_headers[topic])
        #check_time_consistency(topic_headers[topic], topic_times[topic])
        #check_times(topic_headers[topic], topic_times[topic])
        if args.plot:
            plot_times(topic, topic_headers[topic], topic_times[topic])

    for topic in topic_count:
        if topic_count[topic] is 1:
            one_msg_topics.append(topic)
            continue
        print_highlight (topic + ": %d messages without header" % topic_count[topic])

    print_highlight ("There are other %d topics each having only one message" % len(one_msg_topics))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", '--verbose', dest='verbose', action='store_true')
    parser.add_argument("-p", '--plot', dest='plot', action='store_true')
    parser.set_defaults(verbose=False)
    parser.set_defaults(plot=False)
    args, left = parser.parse_known_args()
    if len(left) < 1:
        print ("Usage: " + sys.argv[0] + " [-v] filename [more ...]")
        exit()
    for bag in left:
        check_bag(rosbag.Bag(bag))

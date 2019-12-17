#!/usr/bin/env python2
"""
1. truncate bag by filtering out messages out of a given timestamp range
2. add a bias to all unix stamps in the bag
3. remove pose stream from T265
"""
import rospy
import rosbag
import yaml
import sys
import os
from rospy.rostime import Time, Duration

def main():
    infolder = '.'
    outfolder = 'trunc'
    if not os.path.exists(outfolder):
        os.mkdir(outfolder)
    files = ['synced.bag', 'record-t265.bag', 'record-d400.bag'] if len(sys.argv) == 1 else sys.argv[1:]

    config = yaml.safe_load(open("trunc.yaml"))
    bias = config['bias']
    tmin = config['start'] if 'start' in config else 0
    tmax = config['end'] if 'end' in config else float('inf')

    if True:
        tstart = get_start_time_from_rs_bag('record-d400.bag')
        with rosbag.Bag(infolder + '/' + files[0]) as refbag:
            tend = refbag.get_end_time()
        if (tmin != 0 and tmin < 1e9) or (tmax < 1e9): # relative time
            rmin = tmin
            rmax = tmax
            tmin += tstart
            tmax += tstart
        else:
            rmin = tmin - tstart if tmin != 0 else 0
            rmax = tmax - tstart
        print 'Truncate from %f to %f / %f to %f (%f sec)' % \
            (rmin, rmax, tmin, tmax, min([tmax, tend]) - max([tmin, tstart]))
        print 'With bias %f: %f to %f' % (bias, tmin + bias, tmax + bias)

    for file in files:
        infile = infolder + '/' + file
        with rosbag.Bag(infile) as inbag:
            outfile = outfolder + '/' + file
            outbag = rosbag.Bag(outfile, 'w')
            filter(tmin, tmax, inbag, outbag, bias)
            outbag.close()
            outbag = rosbag.Bag(outfile)
            print ("-------------------- Input: %s ------------------------------" % infile)
            print (inbag)
            print ("------------------- Output: %s ------------------------------" % outfile)
            print (outbag)

def get_start_time_from_rs_bag(file):
    bag = rosbag.Bag(file)
    for topic, msg, t in bag.read_messages():
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.to_sec()
            if is_unix_time(stamp):
                return stamp
            else:
                continue

def is_unix_time(sec):
    return sec > 1e9 and sec < 2e9

def is_rs_topic(topic):
    return topic.startswith('/device_') or topic == '/file_version'

def is_rs_data(topic):
    return topic.startswith('/device_') and topic.endswith('/data')

def is_rs_metadata(topic):
    return topic.startswith('/device_') and (topic.endswith('/metadata') or topic.endswith('/Frame Corrupted'))

def is_rs_notification(topic):
    return topic.startswith('/device_') and '/notification' in topic

def is_rs_pose(topic):
    return topic.startswith('/device_') and 'Pose_0' in topic

def rs_metadata_to_data(topic):
    return topic.rstrip('metadata') + ('transform/data' if 'pose' in topic else 'data')

def filter(start, end, inbag, outbag, bias):
    dic_keep = {}
    dic_pass = {}
    meta_queue = set()
    for topic,msg,t in inbag.read_messages():
        keep = None
        # remove realsense pose stream
        if is_rs_pose(topic):
            continue
        # remove all notification messages, including Frame Corrupted
        elif is_rs_notification(topic):
            print ('Discarded following message on %s at t=%f' % (topic, t.to_sec()))
            print msg
            continue
        # decide whether to keep realsense metadata according to corresponding data
        elif is_rs_metadata(topic):
            data_topic = rs_metadata_to_data(topic)
            #print ('%s: %f' % (topic, t.to_sec()))
            if data_topic in dic_keep and t in dic_keep[data_topic]:
                keep = True
            elif data_topic in dic_pass and t in dic_pass[data_topic]:
                keep = False
            else:
                print ('%s metadata arrived before data!' % topic)
                meta_queue.add((topic, t, msg))
                keep = False
                #exit()
        # keep all realsense information message
        elif is_rs_topic(topic) and not is_rs_data(topic):
            keep = True
        elif hasattr(msg,'header'):
            time = msg.header.stamp.to_sec()
        elif hasattr(msg, 'transforms') and len(msg.transforms) == 1:
            time = msg.transforms[0].header.stamp.to_sec()
        else:
            print msg
            print ('Unsupported message type %s on topic %s' % (msg._type, topic))
            exit()
        if keep is None:
            if not is_unix_time(time):
                print msg
                print ('Invalid stamp %.9f on topic %s' % (time, topic))
                exit()
            keep = (time >= start and time <= end)
        if is_rs_data(topic):
            #print ('%s: %s %f' % (topic, 'keep' if keep else 'pass', t.to_sec()))
            if keep:
                dic_keep.setdefault(topic, []).append(t)
            else:
                dic_pass.setdefault(topic, []).append(t)
        if keep:
            write_with_offset(outbag, topic, msg, t, bias)
        meta_keep = [meta for meta in meta_queue if rs_metadata_to_data(meta[0]) in dic_keep and meta[1] in dic_keep[rs_metadata_to_data(meta[0])]]
        meta_pass = [meta for meta in meta_queue if rs_metadata_to_data(meta[0]) in dic_pass and meta[1] in dic_pass[rs_metadata_to_data(meta[0])]]
        for meta in meta_keep:
            write_with_offset(outbag, meta[0], meta[2], meta[1], bias)
            meta_queue.remove(meta)
        for meta in meta_pass:
            meta_queue.remove(meta)
    if len(meta_queue) > 0:
        exit('Error: There are %d metadata msg without corresponding data!' % len(meta_queue))

def write_with_offset(outbag, topic, msg, t, offset_sec):
    def add_offset(t, offset):
        # ignore those which are not unix timestamps
        return t + offset if is_unix_time(t.to_sec()) else t
    try:
        offset = Duration.from_sec(offset_sec)
        stamp = 0
        if hasattr(msg, 'header'):
            msg.header.stamp = add_offset(msg.header.stamp, offset)
        elif hasattr(msg, 'transforms'):
            for id,_ in enumerate(msg.transforms):
                msg.transforms[id].header.stamp = add_offset(msg.transforms[id].header.stamp, offset)
        # for RealSense metadata
        elif hasattr(msg, 'value'):
            try:
                value = int(msg.value)
                if is_unix_time(float(value) / 1000):
                    msg.value = str(value + int(offset_sec * 1000))
            except ValueError:
                try:
                    value = float(msg.value)
                    if is_unix_time(value / 1000):
                        msg.value = '%.6f' % (value + offset_sec * 1000)
                except ValueError:
                    pass
    except TypeError:
        print msg
        print ('Cannot process above message: topic=%s, t=%.9f, offset=%.9f' % (topic, t.to_sec(), offset_sec))
        exit()
    else:
        t = add_offset(t, offset)
        outbag.write(topic, msg, t)

if __name__ == '__main__':
    main()
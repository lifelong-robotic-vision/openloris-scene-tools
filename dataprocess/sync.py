#!/usr/bin/env python2
"""
1. sync between topics
2. add a bias to all unix stamps in the bag (optional)
"""
import rosbag
import sys
import yaml
import os
from rospy.rostime import Time, Duration

def main():
    with open("sync.yaml") as stream:
        config = yaml.safe_load(stream)

    print config
    topics = config['topics']
    tfs = config['tf']
    bias = config['bias'] if 'bias' in config else 0

    folder = 'sync'
    if not os.path.exists(folder):
        os.mkdir(folder)

    files = ['preprocessed.bag'] if len(sys.argv) == 1 else sys.argv[1:]

    for file in files:
        with rosbag.Bag(file) as inbag:
            outfile = folder + '/' + file
            outbag = rosbag.Bag(outfile, 'w')
            for topic,msg,t in inbag.read_messages():
                if topic == '/tf':
                    if len(msg.transforms) != 1:
                        print (msg)
                        print ("Cannot process tf with multiple transforms!")
                    tf = msg.transforms[0]
                    if tf.header.frame_id in tfs and tf.child_frame_id in tfs[tf.header.frame_id]:
                        write_with_offset(outbag, topic, msg, t, bias + tfs[tf.header.frame_id][tf.child_frame_id])
                        #print ("tf %s offset %f" % (tfs[tf.header.frame_id], tfs[tf.header.frame_id][tf.child_frame_id]))
                elif topic in topics:
                    write_with_offset(outbag, topic, msg, t, bias + topics[topic])
                    #print ("topic %s offset %f" % (topic, topics[topic]))
                else:
                    write_with_offset(outbag, topic, msg, t, bias)

            outbag.close()
            outbag = rosbag.Bag(outfile)
            print ("-------------------- Input: %s ------------------------------" % file)
            print (inbag)
            print ("------------------- Output: %s ------------------------------" % outfile)
            print (outbag)


def write_with_offset(outbag, topic, msg, t, offset_sec):
    def is_unix_time(sec):
        return sec > 1e9 and sec < 2e9
    def add_offset(t, offset):
        # ignore those which are not unix timestamps
        return t + offset if is_unix_time(t.to_sec()) else t
    try:
        offset = Duration.from_sec(offset_sec)
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

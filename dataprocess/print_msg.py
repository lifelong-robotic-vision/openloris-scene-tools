#!/usr/bin/env python2
import rosbag
import sys

filename = sys.argv[1]
topics = sys.argv[2:]
with rosbag.Bag(filename) as bag:
    for topic, msg, t in bag.read_messages(topics):
        print('%s @%.7f ----------------------------' % (topic, t.to_sec()))
        print(msg)
        print('Press ENTER to continue')
        while True:
            try:
                raw_input()
                break
            except EOFError:
                pass

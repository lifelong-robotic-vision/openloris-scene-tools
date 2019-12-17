#!/usr/bin/env python2
"""
print the start time of a normal bag or librealsense bag from message headers
"""
import rosbag
import sys

def main():
    files = sys.argv[1:]
    for file in files:
        bag = rosbag.Bag(file)
        tstart = bag.get_start_time()
        tend = bag.get_end_time()
        print(file + ':')
        print('bag time from %f to %f (duration %f)' % (tstart, tend, tend - tstart))
        for topic, msg, t in bag.read_messages():
            if hasattr(msg, 'header'):
                stamp = msg.header.stamp.to_sec()
                if is_unix_time(stamp):
                    break
                else:
                    continue
        print('first stamp   %f' % stamp)
        bag.close()

def is_unix_time(sec):
    return sec > 1e9 and sec < 2e9

if __name__ == '__main__':
    main()

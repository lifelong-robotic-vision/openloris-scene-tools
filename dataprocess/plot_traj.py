#!/usr/bin/env python2
import sys
import rosbag
import matplotlib.pyplot as plt
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", '--align', dest='align', action='store_true')
    parser.set_defaults(align=False)
    args, left = parser.parse_known_args()
    if len(left) < 2:
        print ('Usage: %s [-a] rosbag_file topic1 [topic2, ...]' % sys.argv[0])
        exit()
    tstart = rosbag.Bag(left[0]).get_start_time()
    topics = left[1:]
    trajs = {topic: read_traj(left[0], topic) for topic in topics}
    if args.align:
        for topic in topics[1:]:
            trajs[topic] = align_traj(trajs[topics[0]], trajs[topic])

    for topic in topics:
        if len(trajs[topic]) == 0:
            print ('No message on topic %s' % topic)
            continue
        tt = sorted(trajs[topic].keys())
        tx = [trajs[topic][t][0] for t in tt]
        ty = [trajs[topic][t][1] for t in tt]
        tt = [t - tstart for t in tt]
        plt.subplot(1,2,1)
        plt.plot(tx, ty)
        plt.subplot(2,2,2)
        plt.plot(tt, tx, label='x')
        plt.subplot(2,2,4)
        plt.plot(tt, ty, label='y')
    plt.subplot(1,2,1)
    plt.legend(topics, loc='upper right')
    plt.show()

def msg_to_pose_values(msg):
    # PoseStamped
    if hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):
        t = msg.pose.position
        q = msg.pose.orientation
    # Odometry / PoseWithCovarianceStamped
    elif hasattr(msg, 'pose') and hasattr(msg.pose, 'pose') and hasattr(msg.pose.pose, 'position'):
        t = msg.pose.pose.position
        q = msg.pose.pose.orientation
    # Transform
    elif hasattr(msg, 'translation'):
        t = msg.translation
        q = msg.rotation
    # tf
    elif hasattr(msg, 'transforms') and len(msg.transforms) > 0 and \
        hasattr(msg.transforms[0], 'transform') and hasattr(msg.transforms[0].transform, 'translation'):
        t = msg.transforms[0].transform.translation
        q = msg.transforms[0].transform.rotation
    else:
        print (str(msg))
        exit('Unsupported msg type!')
    return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]

def align_traj(ref_traj, target_traj, interpolation=True):
    """ Note: return positions only!!! """
    from tum_evaluate_tools import associate
    from tum_evaluate_tools import evaluate_ate
    import numpy
    if interpolation:
        matches, ref_traj_interpolated = associate.associate_with_interpolation(ref_traj, target_traj, 0, 0.03)
    else:
        ref_traj_interpolated = ref_traj
        matches = associate.associate(ref_traj_interpolated, target_traj, 0, 0.005)
    first_xyz = numpy.matrix([[float(value) for value in ref_traj_interpolated[a][0:3]] for a,b in matches]).transpose()
    second_xyz = numpy.matrix([[float(value) for value in target_traj[b][0:3]] for a,b in matches]).transpose()
    rot,trans,trans_error = evaluate_ate.align(second_xyz, first_xyz)
    ate = numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
    numpy.set_printoptions(suppress=True, precision=12)
    print (rot)
    print (trans)
    print ('ATE: %f (%d matches)' % (ate, len(matches)))
    target_traj_trans = {t: (numpy.dot(rot, numpy.matrix(p[:3]).transpose()) + trans).ravel().tolist()[0] for t,p in target_traj.items()}
    return target_traj_trans

def read_traj(bagfile, topic, tmin=0, tmax=float('inf'), relative_time=True):
    traj = {}
    with rosbag.Bag(bagfile) as inbag:
        for topic,msg,t in inbag.read_messages([topic]):
            if not hasattr(msg, 'header'):
                ts = t.to_sec()
            else:
                ts = msg.header.stamp.to_sec()
            if (ts < tmin): continue
            if (ts > tmax): break
            traj[ts - tmin] = msg_to_pose_values(msg)
    return traj

if __name__ == '__main__':
    main()

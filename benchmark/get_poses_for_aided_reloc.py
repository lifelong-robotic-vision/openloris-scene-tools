#!/usr/bin/env python2
from __future__ import print_function
import sys
import argparse
import math
import numpy as np
import matplotlib.pyplot as plt
import openloris_tf_data
sys.path.append('../postprocess')
from tum_evaluate_tools import associate
from tum_evaluate_tools import evaluate_ate
from tf import transformations

def parse_input(filename, remove_repeat=True):
    sequences = {}
    info = {}
    with open(filename) as fp:
        last_pose = None
        for line in fp:
            if line.startswith('#') or len(line.strip()) is 0:
                continue
            elif line.startswith('seq:'):
                try:
                    seq = int(line.lstrip('seq:'))
                    sequences[seq] = {}
                    sequences[seq]['traj'] = {}
                except ValueError:
                    exit('Invalid seq value:\n' + line)
            elif line.startswith('reloc:'):
                aided_reloc = line.lstrip('aided_reloc:')
                sequences[seq]['aided_reloc'] = bool(aided_reloc) \
                    if 'false' not in aided_reloc and 'False' not in aided_reloc else False
            elif ':' in line:
                key = line.split(':')[0].strip()
                value = line.split(':')[1].strip()
                if key not in info: info[key] = value
            else:
                s = line.split()
                if len(s) != 8 and len(s) != 9:
                    exit('Invalid line:\n' + line)
                try:
                    stamp = float(s[0])
                    pose = [float(v) for v in s[-7:]]
                except ValueError:
                    exit('Invalid line:\n' + line)
                else:
                    if np.isnan(sum(pose)): continue
                    if remove_repeat and pose == last_pose: continue
                    sequences[seq]['traj'][stamp] = pose
                    last_pose = pose
    return info, sequences

def tf_matrix_to_values(matrix):
    t = list(matrix[:3, 3])
    q = list(transformations.quaternion_from_matrix(matrix))
    return t + q

def tf_values_to_matrix(values):
    angles = transformations.euler_from_quaternion(values[3:])
    return transformations.compose_matrix(angles=angles, translate=values[:3])

def transform_target_frame(sequences, scene, current_frame, target_frame):
    if current_frame == target_frame:
        return
    tf_target_base = openloris_tf_data.get_tf_values(scene, current_frame, target_frame)
    tf_target_base = tf_values_to_matrix(tf_target_base)
    for seq in sequences:
        for t in sequences[seq]['traj'].keys():
            base_pose = np.dot(tf_values_to_matrix(sequences[seq]['traj'][t]), tf_target_base)
            sequences[seq]['traj'][t] = tf_matrix_to_values(base_pose)

def transform_world_frame(sequences, gts, auto_scale):
    seq = min(sequences.keys())
    traj0 = sequences[seq]['traj']
    gt0 = gts[seq]['traj']
    matches, ref_traj_interpolated = associate.associate_with_interpolation(gt0, traj0, 0, 0.5)
    if len(matches) < 1:
        sequences[seq]['ate'] = {}
        sequences[seq]['ate_rmse'] = float('nan')
        sequences[seq]['ate_num'] = 0
        sequences[seq]['oe'] = {}
        sequences[seq]['aoe_rmse'] = float('nan')
        return
    first_xyz = np.matrix([[float(value) for value in ref_traj_interpolated[a][0:3]] for a,b in matches]).transpose()
    second_xyz = np.matrix([[float(value) for value in traj0[b][0:3]] for a,b in matches]).transpose()
    if auto_scale:
        rot, trans, scale, ate = evaluate_ate.umeyama_align(second_xyz, first_xyz)
    else:
        rot, trans, ate = evaluate_ate.align(second_xyz, first_xyz)
        scale = 1
    sequences[seq]['ate'] = {m[1]: e for m, e in zip(matches, ate)}
    #print sequences[seq]['ate']
    sequences[seq]['ate_rmse'] = np.sqrt(np.dot(ate, ate) / len(ate))
    sequences[seq]['ate_num'] = len(ate)
    tf_matrix = evaluate_ate.compose_transform_matrix(rot, trans, scale)
    for seq in sequences:
        traj_trans = {t: tf_matrix_to_values(np.dot(tf_matrix, tf_values_to_matrix(p))) for t,p in sequences[seq]['traj'].items()}
        sequences[seq]['traj'] = traj_trans
    seq = min(sequences.keys())
    sequences[seq]['oe'] = {b: angle_diff_from_quaternions(ref_traj_interpolated[a][3:], sequences[seq]['traj'][b][3:]) for a,b in matches}
    aoe = np.abs(np.array(sequences[seq]['oe'].values()))
    sequences[seq]['aoe_rmse'] = np.sqrt(np.dot(aoe, aoe) / len(aoe)) if len(aoe) > 0 else float('nan')
    #print(sequences[seq]['oe'].values())

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-g', '--gt-path', help='path to ground-truth files', type=str, default='data/')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-r', '--remove-repeated-pose', help='ignore repeated poses', dest='remove_repeat', action='store_true')
    group.add_argument('-k', '--keep-repeated-pose', help='keep repeated poses', dest='remove_repeat', action='store_false')
    parser.add_argument('-t', '--ate-threshold', type=float, help='ATE threshold of correctness (meter)', default=float('inf'))
    parser.add_argument('-o', '--aoe-threshold', type=float, help='AOE threshold of correctness (degree)', default=float('inf'))
    parser.add_argument('-m', '--max-pose-interval', help='consider lost after no pose for such time (sec)', type=float, default=1.)
    parser.add_argument('-f', '--reloc-score-factor', help='a factor to score time for re-localization (sec)', type=float, default=60.)
    parser.add_argument('-s', '--scale', help='find optimal scale', dest='scale', action='store_true')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-p', '--plot', help='plot trajectories', dest='plot', action='store_true')
    group.add_argument('-np', '--no-plot', help='not plot trajectories', dest='plot', action='store_false')
    parser.set_defaults(remove_repeat=True)
    parser.set_defaults(scale=False)
    parser.set_defaults(plot=True)
    args, left = parser.parse_known_args()
    if len(left) < 1:
        parser.print_help(sys.stderr)
    for filename in left:
        info, sequences = parse_input(filename, args.remove_repeat)
        if 'scene' not in info:
            exit('Please add "scene: xxxxx" into the file')
        print('Got %d trajectories of %s:' % (len(sequences), info['scene']))
        gt_file = args.gt_path + '/gt_%s_1.txt' % info['scene']
        gt_info, gts = parse_input(gt_file)
        print('"%s": {\\' % info['scene'])
        for seq in range(1, max(gts.keys()) + 1):
            traj = gts[seq]['traj']
            p0 = tf_values_to_matrix(traj[min(traj.keys())])
            if seq == 1:
                invp1 = transformations.inverse_matrix(p0)
            rpose = tf_matrix_to_values(np.dot(invp1, p0))
            print('    %d: (%s),' % (seq, ','.join(['%f' % v for v in rpose])))

if __name__ == "__main__":
    main()

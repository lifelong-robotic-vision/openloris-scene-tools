#!/usr/bin/env python2
import sys
import argparse
import os
import copy
import math
import pickle
import matplotlib.pyplot as plt
import evaluate

def update_results(path, gt_dict, seqs):
    result_items = ['ate', 'ate_rmse', 'ate_num', 'c_ate_rmse', 'c_ate_num', 'reloc_time', 'reloc_correct', 'track_time', 'track_cr']
    results = {}
    scenes = ['office']
    algorithms = os.listdir(path)
    for alg in algorithms:
        try:
            files = os.listdir(path + '/' + alg)
        except OSError:
            continue
        auto_scale = alg == 'svo' or alg == 'dso'
        results[alg] = {}
        for file in files:
            if not file.endswith('.txt'): continue
            if not '1-%d-1-%d' % (seqs[0], seqs[1]) in file.replace('_', '-'): continue
            filename = path + '/' + alg + '/' + file
            print('-----------------------------------')
            print(filename + ':')
            try:
                info, sequences = evaluate.parse_input(filename, args.remove_repeat)
                scene = [s for s in scenes if s in info['scene']][0]
                gts = copy.deepcopy(gt_dict[scene]['gts'])
                gt_info = gt_dict[scene]['info']
                evaluate.evaluate(sequences, info, gts, gt_info, args.ate_threshold, args.max_pose_interval, auto_scale)
            except Exception as ex:
                print('Invalid result file')
                print(ex)
            else:
                for seq in sequences:
                    if scene in results[alg] and seq in results[alg][scene]:
                        exit('Multiple results found for %s %s-%d' % (alg, scene, seq))
                    for item in result_items:
                        results[alg].setdefault(scene, {}).setdefault(seq, {})[item] = sequences[seq][item]
    return results

def evaluate_all_per_seq(path, print_per_seq, print_per_scene, print_mean, latex_format, plot):
    """
    path can either be a folder with algorithm outputs or a pickle file with dumped results
    """
    scenes = ['office']
    seqs = [2, 7]
    scene = scenes[0]
    if path.endswith('pkl'):
        with open(path) as f:
            results = pickle.load(f)
    else:
        gt_dict = {}
        gt_file = 'data/gt_%s_1.txt' % scene
        gt_info, gts = evaluate.parse_input(gt_file)
        gt_dict.setdefault(scene, {})['gts'] = gts
        gt_dict[scene]['info'] = gt_info
        results = update_results(path, gt_dict, seqs)
        with open('office-%d-%d.pkl' % (seqs[0], seqs[1]), 'wb') as output:
            pickle.dump(results, output, pickle.HIGHEST_PROTOCOL)
    algorithms = sorted(results.keys())
    algorithms = ['orbslam2_t265', 'maslam', 'ds-slam', \
            'vins_mono_color_imu_with_loop', 'vins_mono_fisheye_imu_with_loop', 'infinitam_aligned_depth',]
    alg_names = ['ORB-s', 'ORB-d', 'DS-SLAM',\
            'VINS-c', 'VINS-f', 'ITAM']
    is_pose_correct = lambda ate: ate <= args.ate_threshold

    max_alg_len = max([len(alg) for alg in algorithms])
    column1_fmt = '%%-%ds' % max_alg_len
    if latex_format:
        num_fmt = ' & %6.3f'
        str_fmt = ' & %6s'
        perc_fmt = ' &%5.1f\\%%'
        line_end = ' \\\\\n'
    else:
        num_fmt = ' %8f'
        str_fmt = ' %8s'
        perc_fmt = ' %8.3f%%'
        line_end = '\n'
    res_reloc = '%d-%d ' % (seqs[0], seqs[1])
    res_score = res_reloc
    for alg in algorithms:
        if not (alg in results and 'office' in results[alg] and seqs[1] in results[alg]['office']):
            res_reloc += str_fmt % 'NO!'
            res_score += str_fmt % 'NO!'
            continue
        res = results[alg]['office'][seqs[1]]
        reloc_time = res['reloc_time'] if (len(res['ate'].keys()) > 0 and is_pose_correct(res['ate'][min(res['ate'].keys())])) else float('inf')
        res_reloc += num_fmt % reloc_time
        res_score += num_fmt % math.exp(reloc_time / (-60.))
        continue

    print('==============================')
    print('    ' + ''.join([str_fmt % alg[:6] for alg in alg_names]))
    print(res_reloc)
    print(res_score)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--remove-repeated-pose', help='ignore repeated poses', dest='remove_repeat', action='store_true')
    parser.add_argument('-k', '--keep-repeated-pose', help='keep repeated poses', dest='remove_repeat', action='store_false')
    parser.add_argument('-t', '--ate-threshold', type=float, help='ATE threshold of correctness', default=0.3)
    parser.add_argument('-m', '--max-pose-interval', type=float, help='consider lost after no pose for such time (sec)', default=1.)
    parser.add_argument('--print-seq', help='print each seq', dest='print_seq', action='store_true')
    parser.add_argument('--no-print-seq', help='print each seq', dest='print_seq', action='store_false')
    parser.add_argument('--print-scene', help='print each scene', dest='print_scene', action='store_true')
    parser.add_argument('--no-print-scene', help='print each scene', dest='print_scene', action='store_false')
    parser.add_argument('--print-mean', help='print total mean', dest='print_mean', action='store_true')
    parser.add_argument('--no-print-mean', help='print total mean', dest='print_mean', action='store_false')
    parser.add_argument('-l', '--latex', help='print in latex table format', dest='latex', action='store_true')
    parser.add_argument('-p', '--plot', help='plot trajectories', dest='plot', action='store_true')
    parser.add_argument('-np', '--no-plot', help='not plot trajectories', dest='plot', action='store_false')
    parser.set_defaults(plot=True)
    parser.set_defaults(remove_repeat=True)
    parser.set_defaults(print_seq=True)
    parser.set_defaults(print_scene=True)
    parser.set_defaults(print_mean=True)
    parser.set_defaults(latex=False)
    args, left = parser.parse_known_args()
    evaluate_all_per_seq(left[0], args.print_seq, args.print_scene, args.print_mean, args.latex, args.plot)

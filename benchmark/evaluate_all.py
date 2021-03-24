#!/usr/bin/env python
import sys
import argparse
import os
import copy
import math
import pickle
import matplotlib.pyplot as plt
import evaluate

# ensure proper fonts (no Type 3 fonts) in eps for paper submission
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

def get_ate_threshold(scene):
    if args.ate_threshold == 0:
        return 1.0 if scene in ['office'] else 3.0 if scene in ['home', 'cafe'] else 5.0
    return args.ate_threshold

def update_results(path, scenes):
    gt_dict = {}
    for scene in scenes:
        gt_file = os.path.dirname(__file__) + '/data/gt_%s_1.txt' % scene
        gt_info, gts = evaluate.parse_input(gt_file)
        gt_dict.setdefault(scene, {})['gts'] = gts
        gt_dict[scene]['info'] = gt_info
    result_items = ['ate', 'ate_rmse', 'ate_num', 'c_ate_rmse', 'c_ate_num', 'oe', 'reloc_time', 'reloc_correct', 'track_time', 'track_cr']
    results = {}
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
            filename = path + '/' + alg + '/' + file
            print('-----------------------------------')
            print(filename + ':')
            try:
                info, sequences = evaluate.parse_input(filename, args.remove_repeat)
                scene = [s for s in scenes if s in info['scene']][0]
                gts = copy.deepcopy(gt_dict[scene]['gts'])
                gt_info = gt_dict[scene]['info']
            except Exception as ex:
                print('Invalid result file')
                print(ex)
            else:
                evaluate.evaluate(sequences, info, gts, gt_info, get_ate_threshold(scene), args.aoe_threshold, args.max_pose_interval, args.reloc_score_factor, auto_scale)
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
    scenes = ['office', 'corridor', 'home', 'cafe', 'market']
    seqs = {'office': 7, 'corridor': 5, 'home': 5, 'cafe': 2, 'market': 3}
    if path.endswith('pkl'):
        with open(path) as f:
            results = pickle.load(f)
    else:
        results = update_results(path, scenes)
        with open('results.pkl', 'wb') as output:
            pickle.dump(results, output, pickle.HIGHEST_PROTOCOL)
    algorithms = sorted(results.keys())
    for_paper = False
    if not for_paper:
        alg_names = algorithms
        pass
    elif 'lifelong' not in path:
        algorithms = ['orbslam2_t265', 'orbslam2_rgbd', 'ds-slam', 'dso', \
            'vins_mono_color_imu_with_loop', 'vins_mono_fisheye_imu_with_loop', 'infinitam_aligned_depth', \
            'efusion_aligned_depth', 'odom']
        alg_names = ['ORB_SLAM2 [16]\n(stereo fisheye)', 'ORB_SLAM2 [16]\n(RGB-D)', 'DS-SLAM [18]\n(RGB-D)', 'DSO [17]\n(RGB)', \
            'VINS-Mono [19]\n(RGB+IMU)', 'VINS-Mono [19]\n(fisheye+IMU)', 'InfiniTAMv2 [20]\n(RGB-D)', \
            'ElasticFusion [21]\n(RGB-D)', 'odometry']
    else:
        algorithms = ['orbslam2_t265', 'maslam', 'ds-slam', \
            'vins_mono_color_imu_with_loop', 'vins_mono_fisheye_imu_with_loop', 'infinitam_aligned_depth']#, 'efusion_reloc_aligned_depth']
        alg_names = ['ORB_SLAM2 [16]\n(stereo fisheye)', 'ORB_SLAM2 [16]\n(RGB-D)', 'DS-SLAM [18]\n(RGB-D)',\
            'VINS-Mono [19]\n(RGB+IMU)', 'VINS-Mono [19]\n(fisheye+IMU)', 'InfiniTAMv2 [20]\n(RGB-D)']#, 'ElasticFusion [20]\n(RGB-D)']

    if plot:
        start_position = {}
        gt_tmin = {}
        gt_tmax = {}
        current_position = 0
        #plt.hold(True)
        for scene in scenes:
            gt_file = 'data/gt_%s_1.txt' % scene
            gt_info, gts = evaluate.parse_input(gt_file)
            plt.plot([current_position, current_position], [0.5, len(algorithms) + 1.5], '-k')
            for seq in gts:
                plt.plot(current_position, len(algorithms) + 0.8, 'ok')
                start_position.setdefault(scene, {})[seq] = current_position
                #print('%s-%d: %f' % (scene, seq, current_position))
                gt_tmin.setdefault(scene, {})[seq] = min(gts[seq]['traj'].keys())
                gt_tmax.setdefault(scene, {})[seq] = max(gts[seq]['traj'].keys())
                current_position += max(gts[seq]['traj'].keys()) - min(gts[seq]['traj'].keys())
                #plt.plot([current_position, current_position], [1, len(algorithms)], ':k')
            plt.plot([current_position, current_position], [0.5, len(algorithms) + 1.5], '-k')
            start_position[scene][-1] = current_position
            start_position[scene]['middle'] = (current_position + start_position[scene][1]) * 0.5
            current_position += 20
            plt.plot([start_position[scene][1], start_position[scene][-1]], [len(algorithms) + 0.8] * 2, '-k')
            plt.text(start_position[scene]['middle'], len(algorithms) + 1, scene, horizontalalignment='center')
        is_pose_correct = lambda ate, oe: ate <= get_ate_threshold(scene) and abs(oe) <= args.aoe_threshold
        alg_id = len(algorithms)

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
    res_rmse = ''
    res_crt = ''
    res_cr = ''
    for alg in algorithms:
        title = column1_fmt
        res_rmse += column1_fmt % alg
        res_crt += column1_fmt % alg
        res_cr += column1_fmt % alg
        if print_mean:
            sum_rmse = 0
            sum_rmse_num = 0
            sum_crt = 0
            sum_track_time = 0
        for scene in scenes:
            scene_sum_rmse = 0
            scene_sum_rmse_num = 0
            scene_sum_crt = 0
            scene_sum_track_time = 0
            for seq in range(seqs[scene]):
                seq += 1
                if print_per_seq: title += ' %7s%d' % (scene[:7], seq)
                if not (alg in results and scene in results[alg] and seq in results[alg][scene]):
                    if print_per_seq:
                        res_rmse += str_fmt % 'N/A'
                        res_crt += str_fmt % 'N/A'
                    continue
                if print_per_seq: res_rmse += num_fmt % results[alg][scene][seq]['c_ate_rmse']
                if not math.isnan(results[alg][scene][seq]['c_ate_rmse']):
                    scene_sum_rmse += results[alg][scene][seq]['c_ate_rmse'] * results[alg][scene][seq]['c_ate_num']
                    scene_sum_rmse_num += results[alg][scene][seq]['c_ate_num']
                track_cr = results[alg][scene][seq]['track_cr']
                if print_per_seq: res_crt += perc_fmt % (track_cr * 100)
                if not math.isnan(track_cr):
                    scene_sum_crt += track_cr * results[alg][scene][seq]['track_time']
                    scene_sum_track_time += results[alg][scene][seq]['track_time']
                if plot:
                    ate = results[alg][scene][seq]['ate']
                    oe = results[alg][scene][seq]['oe'] if 'oe' in results[alg][scene][seq] else {t: 0 for t in ate}
                    if len(ate) == 0: continue
                    offset = -gt_tmin[scene][seq] + start_position[scene][seq]
                    tmin = min(ate.keys())
                    tmax = max(max(ate.keys()), gt_tmax[scene][seq])
                    plot = lambda x, linespec: plt.plot([t + offset for t in x], [alg_id] * len(x), linespec)
                    if is_pose_correct(ate[tmin], oe[tmin]):
                        plot([tmin], 'ob')
                    else:
                        plot([tmin], 'xr')
                    stamps = sorted(ate.keys())
                    plot_correct = []
                    plot_wrong = []
                    for t in stamps:
                        next_t = tmax if t == stamps[-1] \
                            else stamps[stamps.index(t) + 1]
                        if next_t <= t: break
                        next_t = min(next_t, t + args.max_pose_interval)
                        if is_pose_correct(ate[t], oe[t]): plot_correct.append([t, next_t])
                        elif next_t != t + args.max_pose_interval: plot_wrong.append([t, next_t])
                    seg = None
                    for tt in plot_correct:
                        if seg is None:
                            seg = tt
                        elif tt[0] == seg[1]:
                            seg[1] = tt[1]
                        else:
                            plot([seg[0], seg[1]], '-b')
                            seg = tt
                    if seg is not None: plot([seg[0], seg[1]], '-b')
                    seg = None
                    for tt in plot_wrong:
                        if seg is None:
                            seg = tt
                        elif tt[0] == seg[1]:
                            seg[1] = tt[1]
                        else:
                            plot([seg[0], seg[1]], '-r')
                            seg = tt
                    if seg is not None: plot([seg[0], seg[1]], '-r')
            if print_per_scene:
                title += str_fmt % scene[:8]
                res_rmse += num_fmt % (scene_sum_rmse / scene_sum_rmse_num) if scene_sum_rmse_num > 0 else str_fmt % 'N/A'
                res_crt += perc_fmt % (scene_sum_crt / scene_sum_track_time * 100) if scene_sum_track_time > 0 else str_fmt % 'N/A'
                scene_duration = start_position[scene][-1] - start_position[scene][1]
                res_cr += perc_fmt % (scene_sum_crt / scene_duration * 100)
            if print_mean:
                sum_rmse += scene_sum_rmse
                sum_rmse_num += scene_sum_rmse_num
                sum_crt += scene_sum_crt
                sum_track_time += scene_sum_track_time
            if plot:
                plt.text(start_position[scene][1], alg_id + 0.1, '%5.1f%%' % (scene_sum_crt / scene_duration * 100), horizontalalignment='left')
                plt.text(start_position[scene][-1], alg_id - 0.4, '%.3f' % (scene_sum_rmse / scene_sum_rmse_num) if scene_sum_rmse > 0 else 'N/A', horizontalalignment='right')
        if plot: alg_id -= 1
        if print_mean:
            title += str_fmt % 'mean'
            res_rmse += num_fmt % (sum_rmse / sum_rmse_num) if sum_rmse_num > 0 else str_fmt % 'N/A'
            res_crt += perc_fmt % (sum_crt / sum_track_time * 100) if sum_track_time > 0 else str_fmt % 'N/A'
            res_cr += 'N/A'
        title += line_end
        res_rmse += line_end
        res_crt += line_end
        res_cr += line_end
    print('==============================')
    print(title % 'ATE RMSE' + res_rmse)
    print('==============================')
    print(title % ('CR-T (%.1f)' % get_ate_threshold(scene)) + res_crt)
    if plot:
        plt.xlim(start_position[scenes[0]][1] - 10, start_position[scenes[-1]][-1] + 1)
        plt.ylim(0.5, len(algorithms) + 1.4)
        #plt.xticks([(start_position[scene][1] + start_position[scene][-1]) * 0.5 for scene in scenes], scenes)
        plt.xticks([], [])
        plt.yticks(range(len(algorithms), 0, -1), alg_names)
        #plt.gca().spines['top'].set_visible(False)
        #plt.gca().spines['bottom'].set_visible(False)
        plt.box('off')
        #plt.gca().xaxis.set_ticks_position('top')
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--remove-repeated-pose', help='ignore repeated poses', dest='remove_repeat', action='store_true')
    parser.add_argument('-k', '--keep-repeated-pose', help='keep repeated poses', dest='remove_repeat', action='store_false')
    parser.add_argument('-t', '--ate-threshold', type=float, help='ATE threshold of correctness (meter)', default=float('inf'))
    parser.add_argument('-o', '--aoe-threshold', type=float, help='AOE threshold of correctness (degree)', default=float('inf'))
    parser.add_argument('-m', '--max-pose-interval', type=float, help='consider lost after no pose for such time (sec)', default=1.)
    parser.add_argument('-f', '--reloc-score-factor', type=float, help='a factor to score time for re-localization (sec)', default=60.)
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

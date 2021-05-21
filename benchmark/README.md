## OpenLORIS-Scene Benchmarking Tools

To calculate the metrics and visualize the results with this tool, the users need to save the localization results into txt files in a particular format, with results for one sequence (e.g. office1-1), or multiple sequences of one scene (e.g. office1-{1-7}), into each txt file.

### File format

The evaluation script must know the name of the scene and the data sequence id to retrieve corresponding ground-truth trajectories (included in this repo). It must also know which sensor frame the reported poses are for. The users should give these information at the beginning of the txt file:

    scene: office (must be one of [office, corridor, cafe, home, market])
    frame: d400_color (which coordinates' pose your algorithm gives, must be one of [d400_color, d400_depth, d400_imu, t265_fisheye1, t265_fisheye2, t265_imu, odom])

Then, there should be estimated poses for one or more sequences, with each pose per line:

    seq: 1
    #format: timestamp, [output time (optional)], position.x, y, z, quaterniond.x, y, z, w
    1560000083.949067 0 0 0 0 0 0 1
    1560000084.282809258 1568975680.56 0.000445 0.001231 -0.009124 0.000734 0.000177 0.000004 1.000000
    ...
    seq: 2
    1560000084.216086388 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
    1560000084.282809258 0.000445 0.001231 -0.009124 0.000734 0.000177 0.000004 1.000000
    ...

If your SLAM system supports ROS, you may use [`openloris_test_ros.py`](https://github.com/lifelong-robotic-vision/openloris-scene-tools/blob/master/benchmark/openloris_test_ros.py) to help do the testing and save the results. (Config your data path in test.yaml and run `python openloris_test_ros.py -f test.yaml')

### Usage

Use `evaluate.py` to evaluate the results of one file:

```
./evaluate.py -t 1 -o 30 my_result.txt
```

With above command, the script should print metric values (ATE RMSE, etc.) for each sequence in the txt file, and show a figure to visualize the trajectories. The arguments `-t` and `-o` specify the ATE and AOE threshold for correctness judgement (infinity by default). Use `evaluate.py -h` to check more available arguments.

Use `evaluate_all.py` to evaluate results of multiple algorithms for multiple scenes under a given folder:
```
./evaluate_all.py all_results/
```

It will iteratively evaluate all the results within each subfolder (representing each algorithm), print all the results, and give a figure of the overall evalution similar to Fig. 2 and Fig. 3 in the [paper](https://arxiv.org/pdf/1911.05603.pdf). Use `evaluate_all.py -h` to check available arguments.

To re-produce the figures in the paper, check out the `icra20` branch which contains our data, and run the evaluation:
```
git checkout icra20
./evaluate_all.py icra20_results/per-seq
./evaluate_all.py -t 0 -o 30 icra20_results/lifelong
```

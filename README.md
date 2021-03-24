# OpenLORIS-Scene Tools

This is a set of software tools with the [OpenLORIS-Scene](https://lifelong-robotic-vision.github.io/dataset/scene) datasets, including

- [Benchmarking Tools](https://github.com/lifelong-robotic-vision/openloris-scene-tools/tree/master/benchmark): Python scripts to analyze localization results for the OpenLORIS-Scene datasets, calculate the metrics and plot figures.
- [Data Processing Tools](https://github.com/lifelong-robotic-vision/openloris-scene-tools/tree/master/dataprocess): Python scripts to process ROS bags, to organize them into the formats of the OpenLORIS-Scene datasets.
- [RealSense Data Recorder](https://github.com/lifelong-robotic-vision/openloris-scene-tools/tree/master/recorder): A C++ program to record data from two RealSense cameras (D435i and T265) simultaneously, with best quality and least frame loss.

With these tools, users should be able to generate OpenLORIS-Scene-compatible data with their own robot, or to benchmark the localization quality of their own SLAM system. Some of the scripts may also serve as simple tools to manipulate general ROS bags, such as [to merge multiple bags into one](https://github.com/lifelong-robotic-vision/openloris-scene-tools/blob/master/dataprocess/merge_bags.py), or to [visualize multiple trajectories in a ROS bag](https://github.com/lifelong-robotic-vision/openloris-scene-tools/blob/master/dataprocess/plot_traj.py).

Note that:

- The scripts were originally developed for Python2. Some have been ported to Python3, but perhaps not all. Feel free to create an issue or pull request if you found Python incompatibility problems.
- The scripts are designed for particular usages for the OpenLORIS-Scene datasets, not for general puropose. They can have unexpected behavior or even damage your files if using improperly.

Please refer to the README of each toolset for their usages, and our [webpage](https://lifelong-robotic-vision.github.io/dataset/scene) for more information.


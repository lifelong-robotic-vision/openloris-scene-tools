1. Preprocess (convert types, filter outliers, rough sync)
```
python preprocess.py 2019-*.bag sync_time.txt
```

2. Run LIDAR SLAM and record the trajectory (not needed if groundtruth is always from mocap)
```
roslaunch hector_mapping mapping_default.launch base_frame:=base_link odom_frame:=base_odom
rosbag record /slam_out_pose
rosbag play preprocessed.bag
mv 2019-RECORDED-BAG hector-pose.bag
```

2. For the corridor scene, the laser poses need to be re-aligned to the mocap world frame
```
align_laser_pose_to_mocap_map.py
mv hector-pose-trans.bag hector-pose.bag
```

3. Align trajectories from scan/camera/mocap/odom to get their clock offsets against camera, and write them into sync.yaml
```
match_traj.py
```
Or if laser pose is not available:
```
match_traj.py --laser-bag ''
```
Or if mocap is not available, match against T265 pose (z-axis backward):
```
match_traj.py --mocap-bag '' --ref t265
```

4. Synchronize all messages
```
sync.py preprocessed.bag hector-pose.bag
merge sync/*
mv merged.bag synced.bag
```

5. Truncate the data and add a manually specified bias (to hide the real time of recording for privacy)
```
printf 'start: 30\nend: 60\nbias: -600' > trunc.yaml # example values, please modify
vi trunc.yaml
trunc.py
```

6. Generate aligned depth images
```
cd trunc
align_depth.py
```

7. Merge required messages into a single bag file
```
cd trunc
final_merge.py
```

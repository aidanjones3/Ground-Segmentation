# Overview

This repo was used as a way to gain further experience in ROS, and develop my C++ and algorithm development skills.
I attempt to implement the algorithm presented in the following paper: https://www.mrt.kit.edu/z/publ/download/Moosmann_IV09.pdf .
I used the KITTI datset found here as my data source: http://www.cvlibs.net/datasets/kitti/index.php .


-----
Implementation Overview
-----

Given the KITTI datset, create a runable ROS node that implements the following:

![alt text](https://github.com/aidanjones3/Ground-Segmentation/blob/master/Ground_Segmentation.png?raw=true)

The neighborhood graph construction is shown as seen below:

![alt text](https://github.com/aidanjones3/Ground-Segmentation/blob/master/Neighborhood_Graph%20_Construction.png)

Descriptions will be updated here as they are settled upon...

-----
Downloading/Building
-----
```
cd /your/catkin_ws/src/
```

Once you are in the right directory, clone this repo using the command below.
```
git clone https://github.com/aidanjones3/Ground-Segmentation.git'
```
The repo can be build by running the following command in your catkin_ws
```
catkin_make
```
-----
Running the ROS Nodes
-----

```
roscore
```
```
rosbag play -l ../ros_bag_files/kitti_2011_09_26_drive_0084_synced.bag 
```
``` 
rosrun ground_classifier GroundClassifier
```

-----
Useful Links
-----
https://github.com/TixiaoShan/LIO-SAM/pull/159
https://github.com/TixiaoShan/LIO-SAM/blob/master/config/doc/kitti2bag/kitti2bag.py

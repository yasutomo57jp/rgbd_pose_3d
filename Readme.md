# 3D Human Pose Estimation from RGB-D image

A simple 3D human pose estimation using an RGB-D sensor such as Realsense.  
Estimated poses are described in the camera coordinate.

## Environment

- ROS Noetic / Ubuntu 20.04 / Python 3
- RGB-D sensor (A Realsense sensor is recommended.)

## Install

This code uses openpifpaf to estimate 2D human pose, and then estimate the 3D pose by referring the corresponding depth image.
It uses [openpifpaf_ros](https://github.com/yasutomo57jp/openpifpaf_ros) as an implementation of openpifpaf for ROS.

```shell
cd catkin_ws/src
git clone https://github.com/yasutomo57jp/openpifpaf_ros
git clone https://github.com/yasutomo57jp/rgbd_pose_3d
cd ..
catkin_make
```

## requirements for openpifpaf_ros

```shell
pip install openpifpaf
```

## How to RUN

This example uses a Realsense (D453i).

```shell
roslaunch rgbd_pose_3d openpifpaf_realsense.launch
```


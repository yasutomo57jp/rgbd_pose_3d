# 3D Human Pose Estimation from RGB-D image

A simple 3D human pose estimation using an RGB-D sensor such as Realsense.  
Estimated poses are described in the camera coordinate.

## install

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


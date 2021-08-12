#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from openpifpaf_ros.msg import Poses, Pose
import message_filters
import numpy as np


class RGBD3DPoseProc(object):
    def __init__(self, in_depth_topic, in_depth_info_topic, in_pose_topic, out_topic):
        self.bridge = CvBridge()
        sub_depth = message_filters.Subscriber(in_depth_topic, Image, queue_size=1)
        sub_depth_info = message_filters.Subscriber(
            in_depth_info_topic, CameraInfo, queue_size=1
        )
        sub_pose = message_filters.Subscriber(in_pose_topic, Poses, queue_size=1)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sub_depth, sub_depth_info, sub_pose], 100, 0.2
        )
        self.ts.registerCallback(self.callback)
        self.pub = rospy.Publisher(out_topic, Poses, queue_size=1)

    def callback(self, depth, info, poses):
        depth_img = self.bridge.imgmsg_to_cv2(depth)

        poses_3d = []
        for p in poses.poses:
            keypoints = np.asarray(p.keypoints).reshape(-1, 3)  # x, y, conf
            keypoints_pos = np.floor(keypoints).astype(np.int64)

            idx = keypoints_pos[:, 1] * depth_img.shape[1] + keypoints_pos[:, 0]
            flag = idx >= 0
            flag[flag] = idx[flag] < (depth_img.shape[0] * depth_img.shape[1])
            flag[flag] = keypoints[flag, 2] != 0

            depth_values = np.zeros((len(keypoints_pos), 1))
            depth_values[flag] = depth_img.reshape(-1, 1)[idx[flag]]

            xy = (keypoints[:, :2] - np.array([info.K[2], info.K[5]])) / np.array(
                [info.K[0], info.K[4]]
            )
            xy = xy * depth_values

            keypoints_3d = np.hstack((xy, depth_values, keypoints[:, 2:]))
            keypoints_3d[~flag] = np.array([0, 0, 0, 0])

            pose_3d = Pose()
            pose_3d.keypoints = list(keypoints_3d.reshape(-1))
            poses_3d.append(pose_3d)

        self.publish(poses.header, poses_3d)

    def publish(self, header, poses_3d):
        msg = Poses()
        msg.header.stamp = header.stamp
        msg.poses = []

        for p in poses_3d:
            pmsg = Pose()
            pmsg.keypoints = p.keypoints
            msg.poses.append(pmsg)

        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("rgbd_pose_3d", anonymous=True)
    in_depthinfo_topic = rospy.get_param("~in_depthinfo_topic", "/depth/camera_info")
    in_depth_topic = rospy.get_param("~in_depth_topic", "/depth/image_raw")
    in_pose_topic = rospy.get_param("~in_pose_topic", "human_pose")
    out_topic = rospy.get_param("~out_topic", "human_pose_3d")

    rospy.loginfo(in_depth_topic)
    rospy.loginfo(in_depthinfo_topic)
    rospy.loginfo(in_pose_topic)

    recog = RGBD3DPoseProc(in_depth_topic, in_depthinfo_topic, in_pose_topic, out_topic)
    rospy.spin()

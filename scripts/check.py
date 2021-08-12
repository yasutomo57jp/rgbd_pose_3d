#!/usr/bin/env python

import os
import cv2
import numpy as np
import rospy
from openpifpaf_ros.msg import Poses
from geometry_msgs.msg import Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker
import time
import open3d as o3d


class CheckPosesProc(object):
    def __init__(self, in_topic):
        self.sub = rospy.Subscriber(in_topic, Poses, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/pose_vis", MarkerArray, queue_size=10)

    def callback(self, pose):
        ms = MarkerArray()

        i = 0
        for person in pose.poses:
            keypoints = np.array(person.keypoints).reshape(-1, 4)
            keypoints = keypoints[keypoints[:, 3] != 0]

            for k in keypoints:
                m = Marker()
                m.header.stamp = pose.header.stamp
                m.header.frame_id = "camera_color_optical_frame"
                m.id = i
                i += 1

                m.type = Marker.SPHERE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = k[:3] / 100.0
                m.pose.orientation = Quaternion(0, 0, 0, 1)
                m.color.r, m.color.g, m.color.r, m.color.a = [0, 1, 0, 1]
                m.scale = Vector3(0.1, 0.1, 0.1)
                print(m)
                ms.markers.append(m)

        self.pub.publish(ms)


if __name__ == "__main__":
    rospy.init_node("check", anonymous=True)
    in_topic = rospy.get_param("~in_topic", "/human_pose_3d")

    recog = CheckPosesProc(in_topic)
    rospy.spin()

#!/usr/bin/env python3

import rospy
import json
import os
import numpy as np
import cv2
import glob

from pathlib import Path
from cv_bridge import CvBridge
from sensor_msgs.msg import JointState, Image, PointCloud2, CameraInfo
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tf_trans
import open3d as o3d


if __name__ == '__main__':

    rospy.loginfo('waiting for joint_states message...')
    joint_states = rospy.wait_for_message('/joint_states', JointState, timeout=10)
    rospy.loginfo('waiting for depth_img message...')
    depth_img = rospy.wait_for_message('/rgbd/depth/image_raw', Image, timeout=10)
    rospy.loginfo('waiting for rgb_img message...')
    rgb_img = rospy.wait_for_message('/rgbd/rgb/image_raw', Image, timeout=10)
    rospy.loginfo('waiting for samera_info message...')
    camera_info = rospy.wait_for_message('/rgbd/rgb/camera_info', CameraInfo, timeout=10)
    rospy.loginfo('waiting for points message...')
    points = rospy.wait_for_message('/rgbd/depth/points', PointCloud2, timeout=10)

    path = Path('/home/user/dataForCedirnet')
    folders = [f for f in path.iterdir() if f.is_dir()]
    print("Number of folders:", len(folders))

    # --------------------
    # arm_left_joints.json
    # --------------------


    # --------------------
    # arm_left_pose_in_world.json
    # --------------------


    # --------------------
    # arm_left_tcp_pose_in_world.json
    # --------------------


    # --------------------
    # arm_right_joints.json
    # --------------------


    # --------------------
    # arm_right_pose_in_world.json
    # --------------------


    # --------------------
    # arm_right_tcp_pose_in_world.json
    # --------------------


    # --------------------
    # camera_intrinsics.json
    # --------------------


    # --------------------
    # camera_pose_in_world.json
    # --------------------


    # --------------------
    # image_left.png
    # --------------------


    # --------------------
    # point_cloud.ply
    # --------------------


    # --------------------
    # requested_model.json
    # --------------------
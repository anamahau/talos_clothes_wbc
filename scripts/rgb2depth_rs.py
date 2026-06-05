#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from tf_reader import getTfTransform


class RGBDClickNode:
    def __init__(self):

        rospy.init_node('rgbd_click_viewer', anonymous=True)

        self.bridge = CvBridge()

        self.rgb = None
        self.depth = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Subscribers
        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_cb)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_cb)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.info_cb)

        # OpenCV windows
        cv.namedWindow('RGB')
        cv.namedWindow('Depth')
        cv.setMouseCallback('RGB', self.mouse_cb)

        rospy.loginfo('RGBD Click Viewer started')

        self.clickedPoints = []

    def info_cb(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def rgb_cb(self, msg):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(str(e))

    def depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

            # Handle 16UC1 (mm -> m)
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) * 0.001

            self.depth = depth

        except Exception as e:
            rospy.logerr(str(e))

    def mouse_cb(self, event, x, y, flags, param):

        if event != cv.EVENT_LBUTTONDOWN:
            return

        if self.depth is None or self.fx is None:
            print('Waiting for depth or camera info...')
            return

        if y >= self.depth.shape[0] or x >= self.depth.shape[1]:
            return

        z = self.depth[y, x]

        if np.isnan(z) or z <= 0:
            print('[{},{}] Invalid depth'.format(x, y))
            return

        self.clickedPoints.append([x, y])

        X = (x - self.cx) * z / self.fx
        Y = (y - self.cy) * z / self.fy
        Z = z

        transformMatrix_base2orb = getTfTransform('base_link', 'rgbd_depth_optical_frame')
        transformMatrix_orb2rs = np.array([
            [ 0.9989575 , 0.04022338,  0.02158668, -0.00320142],
            [-0.03988856, 0.99908041, -0.0157236 , -0.10964579],
            [-0.02219929, 0.01484615,  0.99964333, -0.05616454],
            [ 0.0       , 0.0       ,  0.0       ,  1.0       ]
        ])
        # transformMatrix_orb2rs = np.linalg.inv(transformMatrix_orb2rs)
        point = np.array([X, Y, Z, 1])
        R_xyz_0 = np.dot(transformMatrix_orb2rs, point)
        R_xyz = np.dot(transformMatrix_base2orb, R_xyz_0)

        print('\nPixel ({}, {}) -> X={:.3f} Y={:.3f} Z={:.3f} m in camera coordinate system'.format(
            x, y, X, Y, Z
        ))
        print('Pixel ({}, {}) -> X={:.3f} Y={:.3f} Z={:.3f} m in Orbec camera coordinate system'.format(
            x, y, R_xyz_0[0], R_xyz_0[1], R_xyz_0[2]
        ))
        print('Pixel ({}, {}) -> X={:.3f} Y={:.3f} Z={:.3f} m in base coordinate system'.format(
            x, y, R_xyz[0], R_xyz[1], R_xyz[2]
        ))

        # Visual feedback
        if self.rgb is not None:
            cv.circle(self.rgb, (x, y), 5, (0, 0, 255), -1)
        if self.depth is not None:
            cv.circle(self.depth, (x, y), 5, (0, 0, 255), -1)

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            if self.rgb is not None:
                for point in self.clickedPoints:
                    cv.circle(self.rgb, (point[0], point[1]), 5, (0, 0, 255), -1)
                cv.imshow('RGB', self.rgb)

            if self.depth is not None:
                depth_vis = cv.normalize(self.depth, None, 0, 255, cv.NORM_MINMAX)
                depth_vis = depth_vis.astype(np.uint8)
                for point in self.clickedPoints:
                    cv.circle(self.depth, (point[0], point[1]), 5, (0, 0, 255), -1)
                cv.imshow('Depth', depth_vis)

            if cv.waitKey(1) == 27:
                break

            rate.sleep()

        cv.destroyAllWindows()


if __name__ == '__main__':
    node = RGBDClickNode()
    node.run()
#!/usr/bin/env python

import os
import rospy
import cv2
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageViewer(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.current_image = None
        self.counter = 0

        self.save_requested = False
        self.save_time = 0

        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.save_dir = rospy.get_param("~save_dir", "./images")

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)

        rospy.loginfo("Press 's' to save current image")
        rospy.loginfo("Press 'q' to quit")

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.current_image is not None:
                cv2.imshow("Image", self.current_image)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('s') and not self.save_requested:
                self.save_requested = True
                self.save_time = time.time() + 5.0
                rospy.loginfo("Saving image in 5 seconds...")

            if self.save_requested and time.time() >= self.save_time:
                filename = os.path.join(
                    self.save_dir,
                    "image_{:06d}.png".format(self.counter)
                )

                rospy.loginfo("Timer expired")

                cv2.imwrite(filename, self.current_image)
                rospy.loginfo("Saved %s", filename)

                self.counter += 1
                self.save_requested = False

            elif key == ord('q'):
                break

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("image_viewer")
    viewer = ImageViewer()
    viewer.run()
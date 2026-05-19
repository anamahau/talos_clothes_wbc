#!/usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import JointState
from wbc_grasp_demo.msg import GraspAction, GraspResult, GraspFeedback

class GraspServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'grasp_action',
            GraspAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )

        self.pub = rospy.Publisher(
            '/whole_body_kinematic_controller/reference_ref',
            JointState,
            queue_size=10
        )

        self.server.start()
        rospy.loginfo("Grasp server started")

    def execute_cb(self, goal):
        rospy.loginfo("Grasp goal: %f", goal.position)

        msg = JointState()
        msg.name = ['gripper_right_joint']
        msg.position = [goal.position]

        rate = rospy.Rate(10)

        for i in range(20):
            self.pub.publish(msg)

            fb = GraspFeedback()
            fb.feedback = goal.position
            self.server.publish_feedback(fb)

            rate.sleep()

        result = GraspResult()
        result.success = True
        self.server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("grasp_server")
    GraspServer()
    rospy.spin()
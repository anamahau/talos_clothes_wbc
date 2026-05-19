#!/usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import JointState
from wbc_grasp_demo.msg import GraspAction, GraspResult, GraspFeedback

class GraspActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'grasp_action',
            GraspAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )

        self.pub = rospy.Publisher(
            '/whole_body_kinematic_controller/reference',
            JointState,
            queue_size=10
        )

        self.server.start()
        rospy.loginfo("Grasp Action Server Started")

    def execute_cb(self, goal):
        rospy.loginfo("Received goal: %f", goal.position)

        joint_msg = JointState()
        joint_msg.name = ['gripper_right_joint']
        joint_msg.position = [goal.position]

        rate = rospy.Rate(10)
        for i in range(10):
            self.pub.publish(joint_msg)

            feedback = GraspFeedback()
            feedback.feedback = goal.position
            self.server.publish_feedback(feedback)

            rate.sleep()

        result = GraspResult()
        result.success = True
        self.server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('grasp_action_server')
    server = GraspActionServer()
    rospy.spin()
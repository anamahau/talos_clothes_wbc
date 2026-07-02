#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('head_joint_terminal_control')

    pub = rospy.Publisher(
        # '/whole_body_kinematic_controller/reference_ref',
        '/whole_body_kinematic_controller/head_joints/reference_ref',
        JointState,
        queue_size=10
    )

    rate = rospy.Rate(10)

    msg = JointState()
    msg.name = ['head_1_joint', 'head_2_joint']

    print("Enter joint values (in radians). Type 'q' to quit.\n")

    while not rospy.is_shutdown():
        try:
            user_input = raw_input("head_1_joint head_2_joint: ")

            if user_input.lower() == 'q':
                break

            values = list(map(float, user_input.split()))

            if len(values) != 2:
                print("Please enter exactly 2 values.")
                continue

            msg.header.stamp = rospy.Time.now()
            msg.position = values
            msg.velocity = [0.0, 0.0]
            msg.effort = [0.0, 0.0]

            pub.publish(msg)

            print("Published: {}".format(values))

        except Exception as e:
            print("Error: {}".format(e))

        rate.sleep()

if __name__ == '__main__':
    main()
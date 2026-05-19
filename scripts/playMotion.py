#!/usr/bin/env python

import rospy
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import actionlib


if __name__ == '__main__':

    rospy.init_node('playMotion_node', anonymous=True)

    # Connect to play_motion server
    client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    print('waiting for server...')
    client.wait_for_server()

    # Create goal
    goal = PlayMotionGoal()
    goal.motion_name = 'close_both_grippers'  # or "open_both_grippers"
    goal.skip_planning = True  # true is safe for grippers
    goal.priority = 0

    # Send goal and wait
    client.send_goal(goal)
    client.wait_for_result()
    print('Grippers motion done')
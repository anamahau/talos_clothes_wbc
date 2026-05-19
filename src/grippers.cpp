#include <iostream>
#include <typeinfo>

#include <talos_clothes_wbc/grippers.h>


grippers::grippers(ros::NodeHandle& nh)
{
    right_gripper_pub_ =
        nh.advertise<trajectory_msgs::JointTrajectory>(
            "/right_gripper_controller/command", 1);

    left_gripper_pub_ =
        nh.advertise<trajectory_msgs::JointTrajectory>(
            "/left_gripper_controller/command", 1);
}

bool grippers::openGripper(const std::string& LorR)
{
    return grippers::sendCommand(LorR, OPEN_POS);
}

bool grippers::closeGripper(const std::string& LorR)
{
    return grippers::sendCommand(LorR, CLOSE_POS);
}

bool grippers::sendCommand(const std::string& LorR, double target_pos)
{
    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint point;

    traj.header.stamp = ros::Time(0);

    if (LorR == "R")
    {
        traj.joint_names.push_back("gripper_right_joint");
    }
    else if (LorR == "L")
    {
        traj.joint_names.push_back("gripper_left_joint");
    }
    else
    {
        ROS_ERROR("Invalid gripper selection: use 'L' or 'R'");
        return false;
    }

    point.positions.push_back(target_pos);
    // point.time_from_start = ros::Duration(1.0);
    point.time_from_start = ros::Duration(DURATION);

    // traj.points.push_back(p0);
    traj.points.push_back(point);

    if (LorR == "R")
        right_gripper_pub_.publish(traj);
    else
        left_gripper_pub_.publish(traj);

    ros::Duration(DURATION).sleep();

    return true;
}
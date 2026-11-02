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

bool grippers::closeGripper(const std::string& LorR, const int number)
{
    if (number == 1)
    {
        return grippers::sendCommand(LorR, CLOSE_POS_1);
    }
    else if (number == 2)
    {
        return grippers::sendCommand(LorR, CLOSE_POS_2);
    }
    else if (number == 3)
    {
        return grippers::sendCommand(LorR, CLOSE_POS_3);
    }
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


bool grippers::openBothGrippers()
{
    trajectory_msgs::JointTrajectory right_traj;
    trajectory_msgs::JointTrajectory left_traj;
    trajectory_msgs::JointTrajectoryPoint point;

    // Give both controllers the same future start time.
    const ros::Time start_time = ros::Time::now() + ros::Duration(0.2);

    right_traj.header.stamp = start_time;
    left_traj.header.stamp = start_time;

    right_traj.joint_names.push_back("gripper_right_joint");
    left_traj.joint_names.push_back("gripper_left_joint");

    point.positions.push_back(0.0);
    point.time_from_start = ros::Duration(DURATION);

    right_traj.points.push_back(point);
    left_traj.points.push_back(point);

    right_gripper_pub_.publish(right_traj);
    left_gripper_pub_.publish(left_traj);

    (start_time + ros::Duration(DURATION) - ros::Time::now()).sleep();

    return true;
}
#include <iostream>
#include <typeinfo>

#include <talos_clothes_wbc/torso_move.h>


torsoMove::torsoMove(ros::NodeHandle& nh)
{
    torso_pub_ =
        nh.advertise<trajectory_msgs::JointTrajectory>(
            "/torso_controller/command", 1
        );
}

bool torsoMove::jointsMove(std::vector<double> joints, float duration)
{
    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint p0;

    traj.header.stamp = ros::Time(0);

    traj.joint_names.push_back("torso_1_joint");
    traj.joint_names.push_back("torso_2_joint");

    p0.positions = joints;
    p0.time_from_start = ros::Duration(duration);
    
    traj.points.push_back(p0);
    
    torso_pub_.publish(traj);
    
    ros::Duration(duration).sleep();
    
    return true;
}
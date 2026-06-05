#include <iostream>
#include <typeinfo>

#include <talos_clothes_wbc/head_move.h>


headMove::headMove(ros::NodeHandle& nh)
{
    head_pub_ =
        nh.advertise<sensor_msgs::JointState>(
            "/whole_body_kinematic_controller/reference_ref", 1
            // "/whole_body_kinematic_controller/head_joints/reference_ref", 1
        );
}

bool headMove::jointsMove(std::vector<double> joints, float duration)
{
    sensor_msgs::JointState traj;

    traj.header.stamp = ros::Time(0);

    traj.name.push_back("head_1_joint");
    traj.name.push_back("head_2_joint");

    traj.position = joints;
    traj.velocity = {0.0, 0.0};
    traj.effort = {0.0, 0.0};
        
    head_pub_.publish(traj);
    
    ros::Duration(duration).sleep();
    
    return true;
}
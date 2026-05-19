#pragma once

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>

class torsoMove
{
    public:
        explicit torsoMove(ros::NodeHandle& nh);
        bool jointsMove(std::vector<double> joints, float duration);
    private:
        ros::Publisher torso_pub_;
};
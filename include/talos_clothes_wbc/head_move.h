#pragma once

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class headMove
{
    public:
        explicit headMove(ros::NodeHandle& nh);
        bool jointsMove(std::vector<double> joints, float duration);
    private:
        ros::Publisher head_pub_;
};
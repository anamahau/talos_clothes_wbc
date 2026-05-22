#pragma once

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <string>

class grippers
{
    public:
        explicit grippers(ros::NodeHandle& nh);
        bool openGripper(const std::string& LorR);
        bool closeGripper(const std::string& LorR, const int number);
    private:
        ros::Publisher right_gripper_pub_;
        ros::Publisher left_gripper_pub_;
        constexpr static double OPEN_POS = 0.0;
        constexpr static double CLOSE_POS_1 = -0.44;
        constexpr static double CLOSE_POS_2 = -0.54;
        constexpr static double DURATION = 2.0;
        bool sendCommand(const std::string& LorR, double target_pos);
};
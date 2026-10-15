#pragma once

#include <string>
#include <iostream>
#include <typeinfo>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class torsoMove
{
    public:
        explicit torsoMove(ros::NodeHandle& nh);
        bool jointsMove(std::vector<double> joints, float duration);
        bool waitForState(double timeout = 2.0);
        void sendReference(double torso1, double torso2);
        double getTorso1() const { return jointValues_[2]; }
        double getTorso2() const { return jointValues_[3]; }
    private:
        void jointsStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
        ros::Publisher pub_;
        ros::Subscriber joints_states_sub_;
        std::vector<double> jointValues_{0.0, 0.0, 0.0, 0.0};
        bool received_ = false;
};
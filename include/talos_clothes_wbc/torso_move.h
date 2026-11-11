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
        void sendReference(double head1, double head2, double torso1, double torso2);
        bool resetHeadTorso(double duration = 60.0, double tol = 0.02);
        double getHead1() const { return jointValues_[0]; }
        double getHead2() const { return jointValues_[1]; }
        double getTorso1() const { return jointValues_[2]; }
        double getTorso2() const { return jointValues_[3]; }
    private:
        void jointsStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
        static double minJerk(double tau);
        ros::Publisher pub_;
        ros::Subscriber joints_states_sub_;
        std::vector<double> jointValues_{0.0, 0.0, 0.0, 0.0};
        bool received_ = false;
};
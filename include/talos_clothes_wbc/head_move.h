#pragma once

#include <string>
#include <iostream>
#include <typeinfo>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

class headMove
{
    public:
        explicit headMove(ros::NodeHandle& nh);
        bool jointsMove(std::vector<double> joints, float duration);
        void cedirnetMove(float duration);
    private:
        void cedirnetFinishedCallback(const std_msgs::Bool::ConstPtr& msg);
        void jointsStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
        ros::Publisher head_pub_;
        ros::Publisher data_recorder_trigger_pub_;
        ros::Publisher create_common_json_pub_;
        ros::Subscriber cedirnet_finished_sub_;
        ros::Subscriber joints_states_sub_;
        std_msgs::Bool msgBool;
        bool cedirnetFinished_ = false;
        std::vector<double> jointValues_{0.0, 0.0};
};
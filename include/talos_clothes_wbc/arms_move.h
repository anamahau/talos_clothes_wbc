#pragma once

#ifndef ARMS_MOVE_H
#define ARMS_MOVE_H

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class armsMove
{
public:
    explicit armsMove(ros::NodeHandle& nh);
    bool absoluteMoveR(std::vector<double> pose, bool wait = false);
    bool absoluteMoveL(std::vector<double> pose, bool wait = false);
    bool absoluteMoveBoth(std::vector<double> poseR, std::vector<double> poseL);
    std::vector<double> getJointValuesR();
    std::vector<double> getJointValuesL();
    geometry_msgs::WrenchStamped getRightFT();
    geometry_msgs::WrenchStamped getLeftFT();
    bool getRightGripperPose(geometry_msgs::PoseStamped& pose_out);
    bool getLeftGripperPose(geometry_msgs::PoseStamped& pose_out);
    bool relativeMoveR(const std::vector<double>& delta);
    bool relativeMoveL(const std::vector<double>& delta);
    bool relativeMoveBoth(const std::vector<double>& delatR, const std::vector<double>& deltaL);
    bool moveR(std::vector<double> pose);
    bool moveL(std::vector<double> pose);
    bool forceMove(const float maxForce);
    double computeForceNorm(const geometry_msgs::WrenchStamped& msg);
    bool getPoseFromTF(
        const std::string& target_frame,
        const std::string& source_frame,
        geometry_msgs::PoseStamped& pose_out);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_right_;
    ros::Publisher pub_left_;
    ros::Subscriber sub_right_ft_;
    ros::Subscriber sub_left_ft_;
    geometry_msgs::WrenchStamped right_ft_msg_;
    geometry_msgs::WrenchStamped left_ft_msg_;
    void rightFTcallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void leftFTcallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    bool received_right_ft_ = false;
    bool received_left_ft_ = false;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif
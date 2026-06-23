#pragma once

#include <iostream>
#include <typeinfo>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

#include <talos_clothes_wbc/grippers.h>
#include <talos_clothes_wbc/head_move.h>
#include <talos_clothes_wbc/arms_move.h>
#include "cloth_detection/cloth_detection_depth.h"



bool headMoveDone = false;

bool success;
float x;
const float headDuration = 0.2;
bool firstRight = true;
std_msgs::Int32 msgInt32;
std_msgs::Bool msgBool;
std_msgs::Float32MultiArrayConstPtr msgMax;
std_msgs::Float32MultiArrayConstPtr msgMin;
geometry_msgs::PoseStampedConstPtr msgCedirnet;
std_msgs::Float32ConstPtr msgTableHeight;
std::string firstArm;
std::string secondArm;
std::vector<double> headJointsDown = {0.7, 0.0};
std::vector<double> headJointsDown_rs = {0.75, 0.0};
std::vector<double> headJointsUp = {0.0, 0.0};
std::vector<float> maxPoint;
std::vector<float> minPoint;
float tableHeight;
const float gripperLength = 0.12;
int waitIdx;
std::vector<double> relativePose;
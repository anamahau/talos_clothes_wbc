#include <talos_clothes_wbc/head_move.h>


headMove::headMove(ros::NodeHandle& nh)
{
    head_pub_ =
        nh.advertise<sensor_msgs::JointState>(
            // "/whole_body_kinematic_controller/reference_ref", 1
            "/whole_body_kinematic_controller/head_torso_joints/reference_ref", 1
        );

    data_recorder_trigger_pub_ =
        nh.advertise<std_msgs::Int32>(
            "/data_recorder/trigger2", 1, false
        );
    
    cedirnet_finished_sub_ =
        nh.subscribe(
            "/cedirnet/finished", 1, &headMove::cedirnetFinishedCallback, this
        );

    joints_states_sub_ =
        nh.subscribe(
            "/joint_states", 1, &headMove::jointsStatesCallback, this
        );

    quality_control_trigger_pub_ =
        nh.advertise<std_msgs::Int32>(
            "/quality_control/trigger", 1, false
        );
    
    quality_control_saved_sub_ =
        nh.subscribe(
            "/quality_control/image_saved", 1, &headMove::qualityControlImageSavedCallback, this
        );
}

bool headMove::jointsMove(std::vector<double> joints, float duration)
{
    sensor_msgs::JointState traj;

    traj.header.stamp = ros::Time(0);

    traj.name.push_back("head_1_joint");
    traj.name.push_back("head_2_joint");
    traj.name.push_back("torso_1_joint");
    traj.name.push_back("torso_2_joint");

    traj.position = {joints[0], joints[1], jointValues_[2], jointValues_[3]};
    traj.velocity = {0.0, 0.0, 0.0, 0.0};
    traj.effort = {0.0, 0.0, 0.0, 0.0};
        
    head_pub_.publish(traj);
    
    // ros::Duration(duration).sleep();

    ros::Rate rate(30);

    const double tol = 0.01;
    
    while (ros::ok())
    {
        ros::spinOnce();
        double e1 = std::abs(jointValues_[0] - joints[0]);
        double e2 = std::abs(jointValues_[1] - joints[1]);
        if (e1 < tol && e2 < tol)
        {
            return true;
        }
        rate.sleep();
    }
    
    return false;
}

void headMove::cedirnetFinishedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    cedirnetFinished_ = msg->data;
}

void headMove::jointsStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    jointValues_[0] = msg->position[16]; // head_1_joint
    jointValues_[1] = msg->position[17]; // head_2_joint
    jointValues_[2] = msg->position[30]; // torso_1_joint
    jointValues_[3] = msg->position[31]; // torso_2_joint
}

void headMove::cedirnetMove(float duration)
{
    std::vector<std::vector<double>> headPosiions = {{0.3, 0.0},
                                                     {0.5, 0.0},
                                                     {0.7, 0.0}};

    std_msgs::Int32 msgInt;

    size_t i = 1;

    for (const auto& joints : headPosiions)
    {
        cedirnetFinished_ = false;
        
        jointsMove(joints, duration);

        ROS_INFO("Move done, starting data recording...");
        
        msgInt.data = i;
        data_recorder_trigger_pub_.publish(msgInt);
        ++i;

        ros::Rate rate(10);

        while (ros::ok() && !cedirnetFinished_)
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

    headMove::jointsMove({0.0, 0.0}, duration);

}

bool headMove::qualityControlMove(float duration)
{
    const std::vector<std::vector<double>> headPosiions = {
        {0.25, 0.3}, {0.25, 0.0}, {0.25, -0.3},
        {0.7, -0.3}, {0.7, 0.0}, {0.7, 0.3}
    };

    const double settleTime = 0.5;
    const double imageTimeout = 20.0;

    std_msgs::Int32 msgInt;
    int i = 1;

    for (const auto& joints : headPosiions)
    {
        qualityControlImageSaved_ = false;

        jointsMove(joints, duration);
        ros::Duration(settleTime).sleep();

        ros::Duration(2.0).sleep();

        ROS_INFO("Head move %d/%zu done, triggering quality control image...",
            i, headPosiions.size());

        msgInt.data = i;
        quality_control_trigger_pub_.publish(msgInt);

        ros::Time start = ros::Time::now();
        ros::Rate rate(10);
        while (ros::ok() && !qualityControlImageSaved_)
        {
            ros::spinOnce();
            if ((ros::Time::now() - start).toSec() > imageTimeout)
            {
                ROS_ERROR("Timeout waiting for quality control image %d", i);
                jointsMove({0.0, 0.0}, duration);
                return false;
            }
            rate.sleep();
        }
        ++i;
    }

    jointsMove({0.0, 0.0}, duration);
    return true;
}

void headMove::qualityControlImageSavedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    qualityControlImageSaved_ = msg->data;
}
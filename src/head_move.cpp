#include <talos_clothes_wbc/head_move.h>


headMove::headMove(ros::NodeHandle& nh)
{
    head_pub_ =
        nh.advertise<sensor_msgs::JointState>(
            "/whole_body_kinematic_controller/reference_ref", 1
            // "/whole_body_kinematic_controller/head_joints/reference_ref", 1
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
    jointValues_[0] = msg->position[16];
    jointValues_[1] = msg->position[17];
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
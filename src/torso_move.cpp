#include <talos_clothes_wbc/torso_move.h>


torsoMove::torsoMove(ros::NodeHandle& nh)
{
    // torso_pub_ =
    //     nh.advertise<trajectory_msgs::JointTrajectory>(
    //         "/torso_controller/command", 1
    //     );
    pub_ =
        nh.advertise<sensor_msgs::JointState>(
            "/whole_body_kinematic_controller/reference_ref", 1
            // "/whole_body_kinematic_controller/head_joints/reference_ref", 1
        );

    joints_states_sub_ =
        nh.subscribe(
            "/joint_states", 1, &torsoMove::jointsStatesCallback, this
        );
}

// bool torsoMove::jointsMove(std::vector<double> joints, float duration)
// {
//     trajectory_msgs::JointTrajectory traj;
//     trajectory_msgs::JointTrajectoryPoint p0;

//     traj.header.stamp = ros::Time(0);

//     traj.joint_names.push_back("torso_1_joint");
//     traj.joint_names.push_back("torso_2_joint");

//     p0.positions = joints;
//     p0.time_from_start = ros::Duration(duration);
    
//     traj.points.push_back(p0);
    
//     torso_pub_.publish(traj);
    
//     ros::Duration(duration).sleep();
    
//     return true;
// }

bool torsoMove::jointsMove(std::vector<double> joints, float duration)
{
    sensor_msgs::JointState traj;

    traj.header.stamp = ros::Time(0);

    traj.name.push_back("head_1_joint");
    traj.name.push_back("head_2_joint");
    traj.name.push_back("torso_1_joint");
    traj.name.push_back("torso_2_joint");

    traj.position = {jointValues_[0], jointValues_[1], joints[0], joints[1]};
    traj.velocity = {0.0, 0.0, 0.0, 0.0};
    traj.effort = {0.0, 0.0, 0.0, 0.0};
        
    pub_.publish(traj);
    
    // ros::Duration(duration).sleep();

    ros::Rate rate(30);

    const double tol = 0.01;
    
    while (ros::ok())
    {
        ros::spinOnce();
        double e1 = std::abs(jointValues_[2] - joints[0]);
        double e2 = std::abs(jointValues_[3] - joints[1]);
        if (e1 < tol && e2 < tol)
        {
            return true;
        }
        rate.sleep();
    }
    
    return false;
}

// void torsoMove::jointsStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
// {
//     jointValues_[0] = msg->position[16]; // head_1_joint
//     jointValues_[1] = msg->position[17]; // head_2_joint
//     jointValues_[2] = msg->position[30]; // torso_1_joint
//     jointValues_[3] = msg->position[31]; // torso_2_joint
// }

bool torsoMove::waitForState(double timeout)
{
    ros::Time t0 = ros::Time::now();
    ros::Rate r(50);
    while (ros::ok() && !received_)
    {
        ros::spinOnce();
        if ((ros::Time::now() - t0).toSec() > timeout) return false;
        r.sleep();
    }
    return received_;
}

void torsoMove::sendReference(double torso1, double torso2)
{
    sensor_msgs::JointState traj;
    traj.header.stamp = ros::Time(0);
    traj.name = {"head_1_joint", "head_2_joint", "torso_1_joint", "torso_2_joint"};
    traj.position = {jointValues_[0], jointValues_[1], torso1, torso2};
    traj.velocity = {0.0, 0.0, 0.0, 0.0};
    traj.effort   = {0.0, 0.0, 0.0, 0.0};
    pub_.publish(traj);
}

// Look up joints by name instead of by hardcoded index
void torsoMove::jointsStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i)
    {
        const std::string& nm = msg->name[i];
        if      (nm == "head_1_joint")  jointValues_[0] = msg->position[i];
        else if (nm == "head_2_joint")  jointValues_[1] = msg->position[i];
        else if (nm == "torso_1_joint") jointValues_[2] = msg->position[i];
        else if (nm == "torso_2_joint") jointValues_[3] = msg->position[i];
    }
    received_ = true;
}
#include "talos_clothes_wbc/arms_move.h"

armsMove::armsMove(ros::NodeHandle& nh)
    : nh_(nh),
      tf_listener_(tf_buffer_)
{
    pub_right_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/whole_body_kinematic_controller/right_gripper_pose/reference_ref",
        1
    );
    pub_left_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/whole_body_kinematic_controller/left_gripper_pose/reference_ref",
        1
    );

    sub_right_ft_ = nh_.subscribe(
        "/right_wrist_ft",
        1,
        &armsMove::rightFTcallback,
        this
    );
    sub_left_ft_ = nh_.subscribe(
        "/left_wrist_ft",
        1,
        &armsMove::leftFTcallback,
        this
    );
}

bool armsMove::moveR(std::vector<double> pose)
{
    if (pose.size() != 7)
    {
        ROS_ERROR("Pose must have 7 elements: qx qy qz qw x y z");
        return false;
    }
    // std::cout << "Hello from absoluteMoveR function!\n";
    // std::cout << "received position: " << pose[4] << ", " << pose[5] << ", " << pose[6] << std::endl;
    // std::cout << "received orientation: " << pose[0] << ", " << pose[1] << ", " << pose[2] << ", " << pose[3] << std::endl;

    geometry_msgs::PoseStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";

    msg.pose.orientation.x = pose[0];
    msg.pose.orientation.y = pose[1];
    msg.pose.orientation.z = pose[2];
    msg.pose.orientation.w = pose[3];

    msg.pose.position.x = pose[4];
    msg.pose.position.y = pose[5];
    msg.pose.position.z = pose[6];

    pub_right_.publish(msg);

    return true;
}

bool armsMove::moveL(std::vector<double> pose)
{
    if (pose.size() != 7)
    {
        ROS_ERROR("Pose must have 7 elements: qx qy qz qw x y z");
        return false;
    }

    geometry_msgs::PoseStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";

    msg.pose.orientation.x = pose[0];
    msg.pose.orientation.y = pose[1];
    msg.pose.orientation.z = pose[2];
    msg.pose.orientation.w = pose[3];

    msg.pose.position.x = pose[4];
    msg.pose.position.y = pose[5];
    msg.pose.position.z = pose[6];

    pub_left_.publish(msg);

    return true;
}

bool armsMove::absoluteMoveR(std::vector<double> pose, bool wait)
{
    bool success = moveR(pose);
    if (!success)
    {
        return false;
    }

    if (wait)
    {
        geometry_msgs::PoseStamped current_pose;

        // target position
        double tx = pose[4];
        double ty = pose[5];
        double tz = pose[6];

        double tolerance = 0.1; // meters
        double timeout = 50.0;   // seconds

        ros::Time start = ros::Time::now();
        ros::Rate rate(50); // 50 Hz

        while (ros::ok())
        {
            // update callbacks (important!)
            ros::spinOnce();

            if (!getRightGripperPose(current_pose))
            {
                ROS_WARN("Waiting for TF...");
                rate.sleep();
                continue;
            }

            double dx = current_pose.pose.position.x - tx;
            double dy = current_pose.pose.position.y - ty;
            double dz = current_pose.pose.position.z - tz;

            double error = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (error < tolerance)
            {
                ROS_INFO("Target reached!");
                // std::cout << "\t target position: " << tx << ", " << ty << ", " << tz << std::endl;
                // std::cout << "\t current position: " << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << std::endl;
                // std::cout << "\t error: " << dx << ", " << dy << ", " << dz << " -> " << error << std::endl;
                return true;
            }

            if ((ros::Time::now() - start).toSec() > timeout)
            {
                ROS_WARN("Timeout: target not reached");
                std::cout << "\t target position: " << tx << ", " << ty << ", " << tz << std::endl;
                std::cout << "\t current position: " << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << std::endl;
                std::cout << "\t error: " << dx << ", " << dy << ", " << dz << " -> " << error << std::endl;
                return false;
            }

            rate.sleep();
        }

        return false;
    }

    return true;
}

bool armsMove::absoluteMoveL(std::vector<double> pose, bool wait)
{
    bool success = moveL(pose);
    if (!success)
    {
        return false;
    }

    if (wait)
    {
        geometry_msgs::PoseStamped current_pose;
    
        // target position
        double tx = pose[4];
        double ty = pose[5];
        double tz = pose[6];
    
        double tolerance = 0.1; // meters
        double timeout = 40.0;   // seconds
    
        ros::Time start = ros::Time::now();
        ros::Rate rate(50); // 50 Hz
    
        while (ros::ok())
        {
            // update callbacks (important!)
            ros::spinOnce();
    
            if (!getLeftGripperPose(current_pose))
            {
                ROS_WARN("Waiting for TF...");
                rate.sleep();
                continue;
            }
    
            double dx = current_pose.pose.position.x - tx;
            double dy = current_pose.pose.position.y - ty;
            double dz = current_pose.pose.position.z - tz;
    
            double error = std::sqrt(dx*dx + dy*dy + dz*dz);
    
            if (error < tolerance)
            {
                ROS_INFO("Target reached!");
                std::cout << "\t target position: " << tx << ", " << ty << ", " << tz << std::endl;
                std::cout << "\t current position: " << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << std::endl;
                std::cout << "\t position error: " << dx << ", " << dy << ", " << dz << " -> " << error << std::endl;
                double dqx = current_pose.pose.orientation.x - pose[0];
                double dqy = current_pose.pose.orientation.y - pose[1];
                double dqz = current_pose.pose.orientation.z - pose[2];
                double dqw = current_pose.pose.orientation.w - pose[3];
                std::cout << "\t target orientation: " << pose[0] << ", " << pose[1] << ", " << pose[2] << ", " << pose[3] << std::endl;
                std::cout << "\t current orientation: " << current_pose.pose.orientation.x << ", " << current_pose.pose.orientation.y << ", " << current_pose.pose.orientation.z << ", " << current_pose.pose.orientation.w << std::endl;
                std::cout << "\t orientation error: " << dqx << ", " << dqy << ", " << dqz << ", " << dqw << " -> " << std::sqrt(dqx*dqx + dqy*dqy + dqz*dqz + dqw*dqw) << std::endl;
                std::cout << "\t time spent: " << (ros::Time::now() - start).toSec() << "s" << std::endl;
                return true;
            }
    
            if ((ros::Time::now() - start).toSec() > timeout)
            {
                ROS_WARN("Timeout: target not reached");
                std::cout << "\t target position: " << tx << ", " << ty << ", " << tz << std::endl;
                std::cout << "\t current position: " << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << std::endl;
                std::cout << "\t error: " << dx << ", " << dy << ", " << dz << " -> " << error << std::endl;
                return false;
            }
    
            rate.sleep();
        }
    
        return false;
    }

    return true;
}

bool armsMove::absoluteMoveBoth(std::vector<double> poseR, std::vector<double> poseL)
{
    bool successR = moveR(poseR);
    bool successL = moveL(poseL);

    if (!successR || !successL)
    {
        return false;
    }

    geometry_msgs::PoseStamped pose_r, pose_l;

    // targets
    double tx_r = poseR[4], ty_r = poseR[5], tz_r = poseR[6];
    double tx_l = poseL[4], ty_l = poseL[5], tz_l = poseL[6];

    double tolerance = 0.05; // meters
    double timeout = 30.0;   // seconds

    ros::Time start = ros::Time::now();
    ros::Rate rate(50);

    while (ros::ok())
    {
        ros::spinOnce();

        bool tf_r = getRightGripperPose(pose_r);
        bool tf_l = getLeftGripperPose(pose_l);

        if (!tf_r || !tf_l)
        {
            ROS_WARN("Waiting for TF...");
            rate.sleep();
            continue;
        }

        // ---- RIGHT ARM ERROR ----
        double dx_r = pose_r.pose.position.x - tx_r;
        double dy_r = pose_r.pose.position.y - ty_r;
        double dz_r = pose_r.pose.position.z - tz_r;
        double err_r = std::sqrt(dx_r*dx_r + dy_r*dy_r + dz_r*dz_r);

        // ---- LEFT ARM ERROR ----
        double dx_l = pose_l.pose.position.x - tx_l;
        double dy_l = pose_l.pose.position.y - ty_l;
        double dz_l = pose_l.pose.position.z - tz_l;
        double err_l = std::sqrt(dx_l*dx_l + dy_l*dy_l + dz_l*dz_l);

        // ---- SUCCESS CONDITION ----
        if (err_r < tolerance && err_l < tolerance)
        {
            ROS_INFO("Both targets reached!");

            // std::cout << "\nRIGHT ARM:\n";
            // std::cout << "\t target: " << tx_r << ", " << ty_r << ", " << tz_r << std::endl;
            // std::cout << "\t current: " << pose_r.pose.position.x << ", " << pose_r.pose.position.y << ", " << pose_r.pose.position.z << std::endl;
            // std::cout << "\t error: " << err_r << std::endl;

            // std::cout << "\nLEFT ARM:\n";
            // std::cout << "\t target: " << tx_l << ", " << ty_l << ", " << tz_l << std::endl;
            // std::cout << "\t current: " << pose_l.pose.position.x << ", " << pose_l.pose.position.y << ", " << pose_l.pose.position.z << std::endl;
            // std::cout << "\t error: " << err_l << std::endl;

            return true;
        }

        // ---- TIMEOUT ----
        if ((ros::Time::now() - start).toSec() > timeout)
        {
            ROS_WARN("Timeout: targets not reached");

            std::cout << "\nRIGHT ARM ERROR: " << err_r << std::endl;
            std::cout << "LEFT ARM ERROR: " << err_l << std::endl;

            return false;
        }

        rate.sleep();
    }

    return false;
}

std::vector<double> armsMove::getJointValuesR()
{
    
    sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh_);
    
    std::vector<double> jointValues(msg->position.begin() + 7, msg->position.begin() + 14);

    return jointValues;
}

std::vector<double> armsMove::getJointValuesL()
{
    
    sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh_);
    
    std::vector<double> jointValues(msg->position.begin(), msg->position.begin() + 7);

    return jointValues;
}

void armsMove::rightFTcallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    right_ft_msg_ = *msg;
    received_right_ft_ = true;
}

void armsMove::leftFTcallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    left_ft_msg_ = *msg;
    received_left_ft_ = true;
}

bool armsMove::getPoseFromTF(
    const std::string& target_frame,
    const std::string& source_frame,
    geometry_msgs::PoseStamped& pose_out)
{
    try
    {
        auto tf = tf_buffer_.lookupTransform(
            target_frame,
            source_frame,
            ros::Time(0),
            ros::Duration(0.05)
        );

        pose_out.header = tf.header;

        pose_out.pose.position.x = tf.transform.translation.x;
        pose_out.pose.position.y = tf.transform.translation.y;
        pose_out.pose.position.z = tf.transform.translation.z;

        pose_out.pose.orientation = tf.transform.rotation;

        return true;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_STREAM("TF lookup failed (" 
                        << target_frame << " <- " << source_frame 
                        << "): " << ex.what());
        return false;
    }
}

bool armsMove::getRightGripperPose(geometry_msgs::PoseStamped& pose_out)
{
    // return getPoseFromTF("base_link", "gripper_right_base_link", pose_out);
    return getPoseFromTF("odom", "gripper_right_base_link", pose_out);
}

bool armsMove::getLeftGripperPose(geometry_msgs::PoseStamped& pose_out)
{
    // return getPoseFromTF("base_link", "gripper_left_base_link", pose_out);
    return getPoseFromTF("odom", "gripper_left_base_link", pose_out);
}

bool armsMove::relativeMoveR(const std::vector<double>& delta, bool wait)
{
    if (delta.size() != 6)
    {
        ROS_ERROR("Delta must have 6 elements: wx wy wz dx dy dz");
        return false;
    }

    geometry_msgs::PoseStamped current_pose;

    if (!getRightGripperPose(current_pose))
    {
        ROS_ERROR("Cannot get current right gripper pose");
        return false;
    }

    geometry_msgs::PoseStamped target = current_pose;

    // std::cout << "orientation: " << target.pose.orientation.x << ", " << target.pose.orientation.y << ", " << target.pose.orientation.z << ", " << target.pose.orientation.w << std::endl;
    std::cout << "start pose (R):  " << target.pose.position.x << ", " << target.pose.position.y << ", " << target.pose.position.z << std::endl;

    // ---- position update ----
    target.pose.position.x += delta[3];
    target.pose.position.y += delta[4];
    target.pose.position.z += delta[5];

    std::cout << "target pose (R): " << target.pose.position.x << ", " << target.pose.position.y << ", " << target.pose.position.z << std::endl;

    // ---- orientation update using exponential map ----
    tf2::Quaternion q_current, q_delta, q_new;
    tf2::fromMsg(current_pose.pose.orientation, q_current);

    // rotation vector (axis * angle)
    tf2::Vector3 omega(delta[0], delta[1], delta[2]);
    double theta = omega.length();

    if (theta < 1e-8)
    {
        // very small rotation -> no change
        q_new = q_current;
    }
    else
    {
        tf2::Vector3 axis = omega.normalized();
        q_delta.setRotation(axis, theta);
        q_new = q_current * q_delta;
        q_new.normalize();
    }

    target.pose.orientation = tf2::toMsg(q_new);

    return absoluteMoveR(
        {
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z
        }, wait
    );
}

bool armsMove::relativeMoveL(const std::vector<double>& delta, bool wait)
{
    if (delta.size() != 6)
    {
        ROS_ERROR("Delta must have 6 elements: wx wy wz dx dy dz");
        return false;
    }

    geometry_msgs::PoseStamped current_pose;

    if (!getLeftGripperPose(current_pose))
    {
        ROS_ERROR("Cannot get current left gripper pose");
        return false;
    }

    geometry_msgs::PoseStamped target = current_pose;

    std::cout << "start pose (L):  " << target.pose.position.x << ", " << target.pose.position.y << ", " << target.pose.position.z << std::endl;

    // ---- position update ----
    target.pose.position.x += delta[3];
    target.pose.position.y += delta[4];
    target.pose.position.z += delta[5];

    std::cout << "target pose (L): " << target.pose.position.x << ", " << target.pose.position.y << ", " << target.pose.position.z << std::endl;

    // ---- orientation update using exponential map ----
    tf2::Quaternion q_current, q_delta, q_new;
    tf2::fromMsg(current_pose.pose.orientation, q_current);

    // rotation vector (axis * angle)
    tf2::Vector3 omega(delta[0], delta[1], delta[2]);
    double theta = omega.length();

    if (theta < 1e-8)
    {
        // very small rotation -> no change
        q_new = q_current;
    }
    else
    {
        tf2::Vector3 axis = omega.normalized();
        q_delta.setRotation(axis, theta);
        q_new = q_current * q_delta;
        q_new.normalize();
    }

    target.pose.orientation = tf2::toMsg(q_new);

    return absoluteMoveL(
        {
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z
        }, wait
    );
}

bool armsMove::relativeMoveBoth(const std::vector<double>& deltaR,
                                const std::vector<double>& deltaL,
                                bool wait)
{
    if (deltaR.size() != 6 || deltaL.size() != 6)
    {
        ROS_ERROR("Both deltas must have 6 elements: wx wy wz dx dy dz");
        return false;
    }

    bool successR = relativeMoveR(deltaR, false);
    bool successL = relativeMoveL(deltaL, wait);

    return successR && successL;
}

bool armsMove::relativeMoveR_force(const std::vector<double>& quaternion, const std::vector<double>& deltaPosition)
{
    geometry_msgs::PoseStamped current_pose;

    if (!getRightGripperPose(current_pose))
    {
        ROS_ERROR("Cannot get current right gripper pose");
        return false;
    }

    geometry_msgs::PoseStamped target = current_pose;

    target.pose.position.x += deltaPosition[0];
    target.pose.position.y += deltaPosition[1];
    target.pose.position.z += deltaPosition[2];

    return absoluteMoveR({
        quaternion[0],
        quaternion[1],
        quaternion[2],
        quaternion[3],
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z
    });
}

bool armsMove::relativeMoveL_force(const std::vector<double>& quaternion, const std::vector<double>& deltaPosition)
{
    geometry_msgs::PoseStamped current_pose;

    if (!getLeftGripperPose(current_pose))
    {
        ROS_ERROR("Cannot get current left gripper pose");
        return false;
    }

    geometry_msgs::PoseStamped target = current_pose;

    target.pose.position.x += deltaPosition[0];
    target.pose.position.y += deltaPosition[1];
    target.pose.position.z += deltaPosition[2];

    return absoluteMoveL({
        quaternion[0],
        quaternion[1],
        quaternion[2],
        quaternion[3],
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z
    });
}

double armsMove::computeForceNorm(const geometry_msgs::WrenchStamped& msg)
{
    const auto& f = msg.wrench.force;
    return std::sqrt(f.x*f.x + f.y*f.y + f.z*f.z);
}

bool armsMove::forceMove(const float maxForce)
{
    ros::Rate rate(50);
    ros::Time start = ros::Time::now();
    double timeout = 10.0;

    double forceR = false;
    double forceL = false;
    bool right_done, left_done;

    while (ros::ok())
    {
        ros::spinOnce();

        if (!received_right_ft_ || !received_left_ft_)
        {
            ROS_WARN("Waiting for FT data...");
            rate.sleep();
            continue;
        }

        forceR = computeForceNorm(right_ft_msg_);
        forceL = computeForceNorm(left_ft_msg_);

        if (!right_done)
        {
            right_done = forceR >= maxForce;
        }
        if (!left_done)
        {
            left_done  = forceL >= maxForce;
        }
        
        // ---- TIMEOUT SAFETY ----
        if ((ros::Time::now() - start).toSec() > timeout)
        {
            ROS_WARN("ForceMove timeout reached");
            return false;
        }
        
        // ---- CONTROL ACTION ----
        if (!right_done)
        {
            // relativeMoveR_force({0.5, 0.5, -0.5, 0.5}, {0.0, -0.05, 0.0});
            relativeMoveR({0.0, 0.0, 0.0, 0.0, -0.5, 0.0});
        }
        else
        {
            std::cout << "Right force reached!" << std::endl;
        }
        
        if (!left_done)
        {
            // relativeMoveL_force({0.5, 0.5, 0.5, -0.5}, {0.0, 0.05, 0.0});
            relativeMoveL({0.0, 0.0, 0.0, 0.0, 0.5, 0.0});
        }
        else
        {
            std::cout << "Left force reached!" << std::endl;
        }

        // ---- STOP CONDITION ----
        if (right_done && left_done)
        {
            ROS_INFO("Force threshold reached for both arms");
            return true;
        }
        
        rate.sleep();
    }

    return false;
}
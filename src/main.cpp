#include <talos_clothes_wbc/main.h>


void doneCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
        headMoveDone = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_script");
    ros::NodeHandle node_handle;

    grippers G(node_handle);
    headMove H(node_handle);
    armsMove A(node_handle);

    // ros::Publisher point_cloud_trigger_pub = node_handle.advertise<std_msgs::Int32>("/PCrequest", 1, true);
    ros::Publisher point_cloud_trigger_pub = node_handle.advertise<std_msgs::Int32>("/PCrequest", 1, false);
    ros::Publisher data_recorder_trigger_pub = node_handle.advertise<std_msgs::Bool>("/data_recorder/trigger", 1, false);

    /* ******************** 1 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 1 ~ move both hands");
    std::cout << "\t Press 1 to move right arm or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.3, -0.4, 0.3};
        success = A.absoluteMoveR(poseR, false);

        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }

    std::cout << "\t Press 1 to move left arm or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.3, 0.4, 0.3};
        success = A.absoluteMoveL(poseL, false);

        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 2 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 2 ~ moving head down");
    std::cout << "\t Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        success = H.jointsMove(headJointsDown, headDuration);
        std::cout << "success: " << success << std::endl;
    
        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 3 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 3 ~ request PC analysis (height of table)");
    std::cout << "\t Press 1 to request PC anaysis or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        msgInt32.data = 3;
        point_cloud_trigger_pub.publish(msgInt32);
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 4 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 4 ~ reading height of table");
    std::cout << "\t rostopic pub /heightOfTable std_msgs/Float32 \"data: 0.0\"\n";
    std::cout << "\t Press 1 to listen on /heightOfTable or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        msgTableHeight = ros::topic::waitForMessage<std_msgs::Float32>("/heightOfTable", node_handle, ros::Duration(10.0));

        if (!msgTableHeight)
        {
            ROS_WARN("Message from /heightOfTable was not received!");
        }
        else
        {
            tableHeight = msgTableHeight->data;
            std::cout << "==== height of table: " << tableHeight << std::endl;
        }
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 5 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 5 ~ request PC analysis");
    std::cout << "\t Press 1 to request PC anaysis or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        msgInt32.data = 1;
        point_cloud_trigger_pub.publish(msgInt32);
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 6 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 6 ~ reading highest point of point cloud");
    std::cout << "\t rostopic pub /highPCpoint std_msgs/Float32MultiArray \"{data: []}\"\n";
    std::cout << "\t Press 1 to listen on /highPCpoint or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        msgMax = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/highPCpoint", node_handle, ros::Duration(10.0));

        if (!msgMax)
        {
            ROS_WARN("Message from /highPCpoint was not received!");
        }
        else
        {
            maxPoint = msgMax->data;
            maxPoint[2] = maxPoint[2] + gripperLength;
            if (maxPoint[1] <= 0)
            {
                firstRight = true;
            }
            else
            {
                firstRight = false;
            }
            std::cout << "==== target point: " << maxPoint[0] << ", " << maxPoint[1] << ", " << maxPoint[2] << std::endl;
        }
    }
    else if (x != 2)
    {
        return 0;
    }
    else
    {
        firstArm = "first";
        secondArm = "second";
    }

    if (firstRight)
    {
        firstArm = "right";
        secondArm = "left";
    }
    else
    {
        firstArm = "left";
        secondArm = "right";
    }

    /* ******************** 7 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 7 ~ moving %s arm", firstArm.c_str());
    if (x == 1 && msgMax)
    {
        std::cout << "\t Press 1 to execute command or 2 to skip this command: ";
        std::cin >> x;

        if (x == 1)
        {
            if (firstRight)
            {
                std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, maxPoint[0]-0.1, maxPoint[1], maxPoint[2]+0.3};
                success = A.absoluteMoveR(poseR);
                firstRight = true;
            }
            else
            {
                std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, maxPoint[0]-0.1, maxPoint[1], maxPoint[2]+0.3};
                success = A.absoluteMoveL(poseL);
                firstRight = false;
            }
            std::cout << "success: " << success << std::endl;
        
            if (!success)
            {
                return 0;
            }
        }
        else if (x != 2)
        {
            return 0;
        }

        std::cout << "\t Press 1 to execute command or 2 to skip this command: ";
        std::cin >> x;

        constexpr double DEG2RAD = M_PI / 180.0;
        
        if (x == 1)
        {
            if (firstRight)
            {
                std::vector<double> poseR = {0, 0, std::sin(45 * DEG2RAD), (-1) * std::sin(45 * DEG2RAD), maxPoint[0], maxPoint[1], maxPoint[2]+0.1};
                success = A.absoluteMoveR(poseR);
                firstRight = true;
            }
            else
            {
                std::vector<double> poseL = {0, 0, std::sin(45 * DEG2RAD), (-1) * std::sin(45 * DEG2RAD), maxPoint[0], maxPoint[1], maxPoint[2]+0.1};
                success = A.absoluteMoveL(poseL);
                firstRight = false;
            }
            std::cout << "success: " << success << std::endl;
        
            if (!success)
            {
                return 0;
            }
        }
        else if (x != 2)
        {
            return 0;
        }
        
        /* ******************** 8 ******************** */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 8 ~ moving %s arm down", firstArm.c_str());
        std::cout << "\t Press 1 to execute command or 2 to skip this command: ";
        std::cin >> x;

        if (x == 1)
        {
            if (firstRight)
            {
                std::vector<double> poseR = {0, 0, std::sin(45 * DEG2RAD), (-1) * std::sin(45 * DEG2RAD), maxPoint[0], maxPoint[1], maxPoint[2]};
                success = A.absoluteMoveR(poseR);
            }
            else
            {
                std::vector<double> poseL = {0, 0, std::sin(45 * DEG2RAD), (-1) * std::sin(45 * DEG2RAD), maxPoint[0], maxPoint[1], maxPoint[2]};
                success = A.absoluteMoveL(poseL);
            }
            std::cout << "success: " << success << std::endl;
        
            if (!success)
            {
                return 0;
            }
        }
        else if (x != 2)
        {
            return 0;
        }
    }
    
    /* ******************** 9 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 9 ~ closing %s gripper", firstArm.c_str());
    std::cout << "\t Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            G.closeGripper("R", 1);
        }
        else
        {
            G.closeGripper("L", 1);
        }
        ros::Duration(2.0).sleep();
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 10 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 10 ~ moving head up");
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        H.jointsMove(headJointsUp, headDuration);
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 11 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 11 ~ moving %s arm up", firstArm.c_str());
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.5, -1 * gripperLength, 0.6};
            success = A.absoluteMoveR(poseR);
        }
        else
        {
            std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.5, gripperLength, 0.6};
            success = A.absoluteMoveL(poseL);
        }
        std::cout << "success: " << success << std::endl;
    
        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 12 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 12 ~ moving head down");
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    // if (x == 1)
    // {
    //     ClothDepthDetector CDD;
    
    //     ros::Publisher motion_pub =
    //         node_handle.advertise<std_msgs::Bool>("/enable_motion", 1, true);
    
    //     ros::Subscriber done_sub =
    //         node_handle.subscribe("/motion_done", 1, doneCallback);
    
    //     ros::Duration(1.0).sleep();

    //     std_msgs::Bool msg;
    //     msg.data = true;
    //     motion_pub.publish(msg);

    //     ROS_INFO("Published: enable_motion = true");

    //     ros::Rate rate(10);

    //     ROS_INFO("Waiting for motion to finish...");

    //     while (ros::ok() && !headMoveDone)
    //     {
    //         ros::spinOnce();
    //         rate.sleep();
    //     }

    //     ROS_INFO("Motion finished -> exiting");
    // }
    if (x == 1)
    {
        success = H.jointsMove(headJointsDown, headDuration);
        std::cout << "success: " << success << std::endl;
    
        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 13 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 13 ~ request PC analysis");
    std::cout << "\t  Press 1 to request PC anaysis or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        msgInt32.data = 2;
        point_cloud_trigger_pub.publish(msgInt32);
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 14 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 14 ~ reading lowest point of point cloud");
    std::cout << "\t  rostopic pub /lowPCpoint std_msgs/Float32MultiArray \"{data: []}\"\n";
    std::cout << "\t  Press 1 to listen on /lowPCpoint or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        msgMin = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/lowPCpoint", node_handle, ros::Duration(10.0));

        if (!msgMin)
        {
            ROS_WARN("Message from /lowPCpoint was not received!");
        }
        else
        {
            minPoint = msgMin->data;
            minPoint[2] = minPoint[2] + 0.05;
            if (firstRight)
            {
                // target point for left
                minPoint[1] = minPoint[1] + gripperLength;
            }
            else
            {
                // target point for right
                minPoint[1] = minPoint[1] - gripperLength;
            }
            std::cout << "==== target point: " << minPoint[0] << ", " << minPoint[1] << ", " << minPoint[2] << std::endl;
        }
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 15 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 15 ~ moving %s arm", secondArm.c_str());
    
    if (x == 1 && msgMin)
    {
        std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
        std::cin >> x;

        if (x == 1)
        {
            if (firstRight)
            {
                geometry_msgs::PoseStamped current_pose;
                A.getLeftGripperPose(current_pose);
                geometry_msgs::Point position = current_pose.pose.position;
                // geometry_msgs::Quaternion orientation = current_pose.pose.orientation;
                // std::cout << "left gripper position: " << position.x << ", " << position.y << ", " << position.z << std::endl;
                // std::cout << "left gripper orientation: " << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << std::endl;
                std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, minPoint[0], position.y, position.z};
                success = A.absoluteMoveL(poseL);
            }
            else
            {
                geometry_msgs::PoseStamped current_pose;
                A.getRightGripperPose(current_pose);
                geometry_msgs::Point position = current_pose.pose.position;
                // geometry_msgs::Quaternion orientation = current_pose.pose.orientation;
                // std::cout << "right gripper position: " << position.x << ", " << position.y << ", " << position.z << std::endl;
                // std::cout << "right gripper orientation: " << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << std::endl;
                std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, minPoint[0], position.y, position.z};
                success = A.absoluteMoveR(poseR);
            }
            std::cout << "success: " << success << std::endl;
        
            if (!success)
            {
                return 0;
            }
        }
        else if (x != 2)
        {
            return 0;
        }

        // std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
        // std::cin >> x;

        // if (x == 1)
        // {
        //     if (firstRight)
        //     {
        //         std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, minPoint[0], minPoint[1] + 0.1, minPoint[2]};
        //         success = A.absoluteMoveL(poseL);
        //     }
        //     else
        //     {
        //         std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, minPoint[0], minPoint[1] - 0.1, minPoint[2]};
        //         success = A.absoluteMoveR(poseR);
        //     }
        //     std::cout << "success: " << success << std::endl;
        
        //     if (!success)
        //     {
        //         return 0;
        //     }
        // }
        // else if (x != 2)
        // {
        //     return 0;
        // }
        
        /* ******************** 16 ******************* */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 16 ~ moving %s arm to the point", secondArm.c_str());
        std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
        std::cin >> x;

        if (x == 1)
        {
            if (firstRight)
            {
                std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, minPoint[0], minPoint[1], minPoint[2]};
                success = A.absoluteMoveL(poseL);
            }
            else
            {
                std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, minPoint[0], minPoint[1], minPoint[2]};
                success = A.absoluteMoveR(poseR);
            }
            std::cout << "success: " << success << std::endl;
        
            if (!success)
            {
                return 0;
            }
        }
        else if (x != 2)
        {
            return 0;
        }
    }
    
    /* ******************** 17 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 17 ~ closing %s gripper", secondArm.c_str());
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            G.closeGripper("L", 2);
        }
        else
        {
            G.closeGripper("R", 2);
        }
        ros::Duration(2.0).sleep();
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 18 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 18 ~ moving %s arm", secondArm.c_str());
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            // std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.5, 0.3, 0.4};
            std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.5, 0.35, minPoint[2] + 0.2};
            success = A.absoluteMoveL(poseL);
        }
        else
        {
            // std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.5, -0.3, 0.4};
            std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.5, -0.35, minPoint[2] + 0.2};
            success = A.absoluteMoveR(poseR);
        }
        std::cout << "success: " << success << std::endl;
    
        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 19 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 19 ~ opening %s gripper", firstArm.c_str());
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            G.openGripper("R");
        }
        else
        {
            G.openGripper("L");
        }
        ros::Duration(2.0).sleep();
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 20 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 20 ~ moving %s arm", firstArm.c_str());
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.4, 0.4};
            success = A.absoluteMoveR(poseR);
        }
        else
        {
            std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.4, 0.4};
            success = A.absoluteMoveL(poseL);
        }
        std::cout << "success: " << success << std::endl;
    
        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }

    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.4, 0.2};
            success = A.absoluteMoveR(poseR);
        }
        else
        {
            std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.4, 0.2};
            success = A.absoluteMoveL(poseL);
        }
        std::cout << "success: " << success << std::endl;
    
        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 21 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 21 ~ moving %s arm", secondArm.c_str());
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.5, 0.1, 0.7};
            success = A.absoluteMoveL(poseL);
        }
        else
        {
            std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.5, -0.1, 0.7};
            success = A.absoluteMoveR(poseR);
        }
        std::cout << "success: " << success << std::endl;
    
        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 22 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 22 ~ moving head down");
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        success = H.jointsMove(headJointsDown_rs, headDuration);
        std::cout << "success: " << success << std::endl;
    
        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 23 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 23 ~ CeDiRNet");
    // std::cout << "\t  Publish CeDiRNet point to /cedirnetPoint and press 1 to continue: ";
    std::cout << "\t  Press 1 to request CeDiRNet analysis or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        // msgBool.data = true;
        // data_recorder_trigger_pub.publish(msgBool);
        H.cedirnetMove(headDuration);
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 24 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 24 ~ CeDiRNet");
    std::cout << "\t  rostopic pub /cedirnet/goal_pose geometry_msgs/PoseStamped \"{pose: {position: {x: , y: , z: }, orientation: {x: , y: , z: , w: }}}\"\n";
    std::cout << "\t  Press 1 to listen on /cedirnet/goal_pose or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        // msgCedirnet = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/cedirnet/goal_pose", node_handle, ros::Duration(10.0));
        msgCedirnet = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/cedirnet/goal_pose", node_handle, ros::Duration(10.0));
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 25 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 25 ~ moving %s arm", firstArm.c_str());
    if (x == 1)
    {
        std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
        std::cin >> x;

        if (x == 1)
        {
            if (firstRight)
            {
                // std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, msgCedirnet->data[0], msgCedirnet->data[1], msgCedirnet->data[2]};
                std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
                // std::vector<double> poseR = {msgCedirnet->pose.orientation.x, msgCedirnet->pose.orientation.y, msgCedirnet->pose.orientation.z, msgCedirnet->pose.orientation.w, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
                success = A.absoluteMoveR(poseR);
            }
            else
            {
                // std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, msgCedirnet->data[0], msgCedirnet->data[1], msgCedirnet->data[2]};
                std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
                // std::vector<double> poseL = {msgCedirnet->pose.orientation.x, msgCedirnet->pose.orientation.y, msgCedirnet->pose.orientation.z, msgCedirnet->pose.orientation.w, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
                success = A.absoluteMoveL(poseL);
            }
            std::cout << "success: " << success << std::endl;
        
            if (!success)
            {
                return 0;
            }
        }
        else if (x != 2)
        {
            return 0;
        }
    }
    
    /* ******************** 26 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 26 ~ closing %s gripper", firstArm.c_str());
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            G.closeGripper("R", 2);
        }
        else
        {
            G.closeGripper("L", 2);
        }
        ros::Duration(2.0).sleep();
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 27 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 27 ~ moving %s arm", secondArm.c_str());
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.2, 0.6};
            success = A.absoluteMoveL(poseL);
        }
        else
        {
            std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.2, 0.6};
            success = A.absoluteMoveR(poseR);
        }
        
        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 28 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 28 ~ moving %s arm", firstArm.c_str());
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        if (firstRight)
        {
            std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.2, 0.6};
            success = A.absoluteMoveR(poseR);
        }
        else
        {
            std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.2, 0.6};
            success = A.absoluteMoveL(poseL);
        }
        
        if (!success)
        {
            return 0;
        }
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 29 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 29 ~ moving both arms (by force)");
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        A.forceMove_old(15);
    }
    else if (x != 2)
    {
        return 0;
    }
    
    /* ******************** 30 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 30 ~ moving both hands down");
    
    float relativeHeight;
    relativeHeight = (0.3 - tableHeight) - 0.20;
    std::cout << "relative move on z: " << relativeHeight << std::endl;
    
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        std::vector<double> relativePose = {0.0, 0.0, 0.0, 0.0, 0.0, -relativeHeight};
        success = A.relativeMoveBoth(relativePose, relativePose);
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 31 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 31 ~ moving both hands forward");
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        std::vector<double> relativePose = {0.0, 0.0, 0.0, 0.3, 0.0, 0.0};
        success = A.relativeMoveBoth(relativePose, relativePose);
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 32 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 32 ~ moving both hands up and backward");
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        std::vector<double> relativePose = {0.0, 0.0, 0.0, -0.15, 0.0, 0.1};
        success = A.relativeMoveBoth(relativePose, relativePose);
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 33 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 33 ~ opening both grippers");
    std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        G.openGripper("R");
        G.openGripper("L");
        ros::Duration(2.0).sleep();
    }
    else if (x != 2)
    {
        return 0;
    }

    /* ******************** 34 ******************* */
    // std::cout << std::endl;
    // ROS_INFO("\nSTEP 34 ~ moving both arms to home pose");
    // std::cout << "\t  Press 1 to execute command or 2 to skip this command: ";
    // std::cin >> x;
    
    // if (x == 1)
    // {
    //     success = A.moveHomeR();
    //     std::cout << "success: " << success << std::endl;
    //     success = A.moveHomeL();
    //     std::cout << "success: " << success << std::endl;
    
    //     if (!success)
    //     {
    //         return 0;
    //     }
    // }
    // else if (x != 2)
    // {
    //     return 0;
    // }

}
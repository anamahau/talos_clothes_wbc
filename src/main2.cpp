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

    /* ******************************************* */
    std::cout << "\nPress 1 to move arms to home pose: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 1 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 1 ~ move both hands");

    std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.3, -0.4, 0.3};
    success = A.absoluteMoveR(poseR, false);
    if (!success)
    {
        return 0;
    }

    ros::Duration(4.0).sleep();

    std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.3, 0.4, 0.3};
    success = A.absoluteMoveL(poseL, true);
    if (!success)
    {
        return 0;
    }

    /* ******************************************* */
    std::cout << "\nPress 1 to start the program: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 2 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 2 ~ moving head down");

    success = H.jointsMove(headJointsDown, headDuration);
    std::cout << "success: " << success << std::endl;
    if (!success)
    {
        return 0;
    }

    /* ******************** 3 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 3 ~ request PC analysis (height of table)");

    msgInt32.data = 3;
    point_cloud_trigger_pub.publish(msgInt32);

    /* ******************** 4 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 4 ~ reading height of table");

    waitIdx = 0;

    while (waitIdx < 4)
    {
        msgTableHeight = ros::topic::waitForMessage<std_msgs::Float32>("/heightOfTable", node_handle, ros::Duration(20.0));
        
        if (!msgTableHeight)
        {
            ROS_WARN("Message from /heightOfTable was not received!");
            waitIdx++;
        }
        else
        {
            tableHeight = msgTableHeight->data;
            std::cout << "==== height of table: " << tableHeight << std::endl;
            waitIdx = 4;
        }   
    }
    if (!msgTableHeight)
    {
        return 0;
    }

    /* ******************** 5 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 5 ~ request PC analysis");

    msgInt32.data = 1;
    point_cloud_trigger_pub.publish(msgInt32);

    /* ******************** 6 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 6 ~ reading highest point of point cloud");

    waitIdx = 0;

    while (waitIdx < 4)
    {
        msgMax = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/highPCpoint", node_handle, ros::Duration(20.0));

        if (!msgMax)
        {
            ROS_WARN("Message from /highPCpoint was not received!");
            waitIdx++;
        }
        else
        {
            maxPoint = msgMax->data;
            maxPoint[2] = maxPoint[2] + gripperLength;
            if (maxPoint[1] <= 0)
            {
                firstRight = true;
                firstArm = "right";
                secondArm = "left";
            }
            else
            {
                firstRight = false;
                firstArm = "left";
                secondArm = "right";
            }
            std::cout << "==== target point: " << maxPoint[0] << ", " << maxPoint[1] << ", " << maxPoint[2] << std::endl;
            waitIdx = 4;
        }
    }
    if (!msgMax)
    {
        return 0;
    }

    /* ******************************************* */
    std::cout << "\nPress 1 to continue the program: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 7 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 7 ~ moving %s arm", firstArm.c_str());

    if (firstRight)
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, maxPoint[0]-0.1, maxPoint[1], maxPoint[2]+0.3};
        success = A.absoluteMoveR(poseR, true);
        firstRight = true;
    }
    else
    {
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, maxPoint[0]-0.1, maxPoint[1], maxPoint[2]+0.3};
        success = A.absoluteMoveL(poseL, true);
        firstRight = false;
    }

    if (!success)
    {
        return 0;
    }

    constexpr double DEG2RAD = M_PI / 180.0;
    
    if (firstRight)
    {
        std::vector<double> poseR = {0, 0, std::sin(45 * DEG2RAD), (-1) * std::sin(45 * DEG2RAD), maxPoint[0], maxPoint[1], maxPoint[2]+0.1};
        success = A.absoluteMoveR(poseR, true);
        firstRight = true;
    }
    else
    {
        std::vector<double> poseL = {0, 0, std::sin(45 * DEG2RAD), (-1) * std::sin(45 * DEG2RAD), maxPoint[0], maxPoint[1], maxPoint[2]+0.1};
        success = A.absoluteMoveL(poseL, true);
        firstRight = false;
    }

    if (!success)
    {
        return 0;
    }
    
    /* ******************** 8 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 8 ~ moving %s arm down", firstArm.c_str());

    if (firstRight)
    {
        std::vector<double> poseR = {0, 0, std::sin(45 * DEG2RAD), (-1) * std::sin(45 * DEG2RAD), maxPoint[0], maxPoint[1], maxPoint[2]};
        success = A.absoluteMoveR(poseR, true);
    }
    else
    {
        std::vector<double> poseL = {0, 0, std::sin(45 * DEG2RAD), (-1) * std::sin(45 * DEG2RAD), maxPoint[0], maxPoint[1], maxPoint[2]};
        success = A.absoluteMoveL(poseL, true);
    }

    if (!success)
    {
        return 0;
    }
    
    /* ******************** 9 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 9 ~ closing %s gripper", firstArm.c_str());

    if (firstRight)
    {
        G.closeGripper("R", 1);
    }
    else
    {
        G.closeGripper("L", 1);
    }
    ros::Duration(2.0).sleep();
    
    /* ******************** 10 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 10 ~ moving head up");

    H.jointsMove(headJointsUp, headDuration);
    if (!success)
    {
        return 0;
    }
    
    /* ******************** 11 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 11 ~ moving %s arm up", firstArm.c_str());

    if (firstRight)
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.5, -1 * gripperLength, 0.6};
        success = A.absoluteMoveR(poseR, true);
    }
    else
    {
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.5, gripperLength, 0.6};
        success = A.absoluteMoveL(poseL, true);
    }

    if (!success)
    {
        return 0;
    }
    
    /* ******************** 12 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 12 ~ moving head down");

    success = H.jointsMove(headJointsDown, headDuration);
    if (!success)
    {
        return 0;
    }
    ros::Duration(headDuration).sleep();
    
    /* ******************** 13 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 13 ~ request PC analysis");

    msgInt32.data = 2;
    point_cloud_trigger_pub.publish(msgInt32);
    
    /* ******************** 14 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 14 ~ reading lowest point of point cloud");

    waitIdx = 0;

    while (waitIdx < 4)
    {
        msgMin = ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/lowPCpoint", node_handle, ros::Duration(20.0));

        if (!msgMin)
        {
            ROS_WARN("Message from /lowPCpoint was not received!");
            waitIdx++;
        }
        else
        {
            minPoint = msgMin->data;
            minPoint[2] = minPoint[2] + 0.05;
            if (firstRight)
            {
                // target point for left
                minPoint[1] = minPoint[1] + gripperLength + 0.05;
            }
            else
            {
                // target point for right
                minPoint[1] = minPoint[1] - gripperLength - 0.05;
            }
            std::cout << "==== target point: " << minPoint[0] << ", " << minPoint[1] << ", " << minPoint[2] << std::endl;
            waitIdx = 4;
        }
    }
    if (!msgMin)
    {
        return 0;
    }

    /* ******************************************* */
    std::cout << "\nPress 1 to continue the program: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */
    
    /* ******************** 15 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 15 ~ moving %s arm", secondArm.c_str());

    if (firstRight)
    {
        geometry_msgs::PoseStamped current_pose;
        A.getLeftGripperPose(current_pose);
        geometry_msgs::Point position = current_pose.pose.position;
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, minPoint[0], position.y, position.z};
        success = A.absoluteMoveL(poseL, true);
    }
    else
    {
        geometry_msgs::PoseStamped current_pose;
        A.getRightGripperPose(current_pose);
        geometry_msgs::Point position = current_pose.pose.position;
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, minPoint[0], position.y, position.z};
        success = A.absoluteMoveR(poseR, true);
    }

    if (!success)
    {
        return 0;
    }
        
    /* ******************** 16 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 16 ~ moving %s arm to the point", secondArm.c_str());

    if (firstRight)
    {
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, minPoint[0], minPoint[1], minPoint[2]};
        success = A.absoluteMoveL(poseL, true);
    }
    else
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, minPoint[0], minPoint[1], minPoint[2]};
        success = A.absoluteMoveR(poseR, true);
    }

    if (!success)
    {
        return 0;
    }
    
    /* ******************** 17 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 17 ~ closing %s gripper", secondArm.c_str());

    if (firstRight)
    {
        G.closeGripper("L", 2);
    }
    else
    {
        G.closeGripper("R", 2);
    }
    ros::Duration(2.0).sleep();

    // TODO: checkGrippingSuccess
    bool gSuccess;
    if (firstRight)
    {
        gSuccess = A.checkGrippingSuccess(15.0, "L");
    }
    else
    {
        gSuccess = A.checkGrippingSuccess(15.0, "R")
    }
    if (gSuccess)
    {
        std::cout << "cloth was successfully gripped\n";
    }
    else
    {
        std::cout << "cloth was not gripped\n";
    }
    
    /* ******************** 18 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 18 ~ moving %s arm", secondArm.c_str());

    if (firstRight)
    {
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.5, 0.35, minPoint[2] + 0.2};
        success = A.absoluteMoveL(poseL);
    }
    else
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.5, -0.35, minPoint[2] + 0.2};
        success = A.absoluteMoveR(poseR);
    }

    if (!success)
    {
        return 0;
    }
    
    /* ******************** 19 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 19 ~ opening %s gripper", firstArm.c_str());

    if (firstRight)
    {
        G.openGripper("R");
    }
    else
    {
        G.openGripper("L");
    }
    ros::Duration(2.0).sleep();
    
    /* ******************** 20 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 20 ~ moving %s arm", firstArm.c_str());

    if (firstRight)
    {
        // std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.4, 0.4};
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.3, 0.4};
        success = A.absoluteMoveR(poseR, true);
    }
    else
    {
        // std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.4, 0.4};
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.3, 0.4};
        success = A.absoluteMoveL(poseL, true);
    }

    if (!success)
    {
        return 0;
    }

    if (firstRight)
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.4, 0.2};
        success = A.absoluteMoveR(poseR, true);
    }
    else
    {
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.4, 0.2};
        success = A.absoluteMoveL(poseL, true);
    }

    if (!success)
    {
        return 0;
    }
    
    /* ******************** 21 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 21 ~ moving %s arm", secondArm.c_str());

    if (firstRight)
    {
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.5, 0.1, 0.7};
        success = A.absoluteMoveL(poseL, true);
    }
    else
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.5, -0.1, 0.7};
        success = A.absoluteMoveR(poseR, true);
    }

    if (!success)
    {
        return 0;
    }

    /* ******************** 22 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 22 ~ moving head down");

    success = H.jointsMove(headJointsDown_rs, headDuration);
    if (!success)
    {
        return 0;
    }
    ros::Duration(headDuration).sleep();
    
    /* ******************** 23 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 23 ~ CeDiRNet");

    H.cedirnetMove(headDuration);
    
    /* ******************** 24 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 24 ~ CeDiRNet");

    waitIdx = 0;

    while (waitIdx < 4)
    {
        msgCedirnet = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/cedirnet/goal_pose", node_handle, ros::Duration(20.0));

        if (!msgCedirnet)
        {
            ROS_WARN("Message from /cedirnet/goal_pose was not received!");
            waitIdx++;
        }
        else
        {
            std::cout << "==== target point: " << msgCedirnet->pose.position.x << ", " << msgCedirnet->pose.position.y << ", " << msgCedirnet->pose.position.z << std::endl;
            waitIdx = 4;
        }
    }
    if (!msgCedirnet)
    {
        return 0;
    }


    /* ******************************************* */
    std::cout << "\nPress 1 to continue the program: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */
    
    /* ******************** 25 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 25 ~ moving %s arm", firstArm.c_str());

    if (firstRight)
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
        // std::vector<double> poseR = {msgCedirnet->pose.orientation.x, msgCedirnet->pose.orientation.y, msgCedirnet->pose.orientation.z, msgCedirnet->pose.orientation.w, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
        success = A.absoluteMoveR(poseR, true);
    }
    else
    {
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
        // std::vector<double> poseL = {msgCedirnet->pose.orientation.x, msgCedirnet->pose.orientation.y, msgCedirnet->pose.orientation.z, msgCedirnet->pose.orientation.w, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
        success = A.absoluteMoveL(poseL, true);
    }

    if (!success)
    {
        return 0;
    }
    
    /* ******************** 26 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 26 ~ closing %s gripper", firstArm.c_str());

    if (firstRight)
    {
        G.closeGripper("R", 2);
    }
    else
    {
        G.closeGripper("L", 2);
    }
    ros::Duration(2.0).sleep();

    /* ******************** 27 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 27 ~ moving %s arm", secondArm.c_str());

    if (firstRight)
    {
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.2, 0.6};
        success = A.absoluteMoveL(poseL, false);
    }
    else
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.2, 0.6};
        success = A.absoluteMoveR(poseR, false);
    }
    
    if (!success)
    {
        return 0;
    }

    ros::Duration(4.0).sleep();

    /* ******************** 28 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 28 ~ moving %s arm", firstArm.c_str());

    if (firstRight)
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.2, 0.6};
        success = A.absoluteMoveR(poseR, true);
    }
    else
    {
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.2, 0.6};
        success = A.absoluteMoveL(poseL, true);
    }
    
    if (!success)
    {
        return 0;
    }

    /* ******************************************* */
    float relativeHeight;
    relativeHeight = (0.6 - tableHeight) - 0.20;
    std::cout << "relative move on z: " << relativeHeight << std::endl;
    std::cout << "\nPress 1 to continue the program: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */
    
    /* ******************** 29 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 29 ~ moving both arms (by force)");

    A.forceMove(10);
    
    /* ******************** 30 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 30 ~ moving both hands down");

    relativePose = {0.0, 0.0, 0.0, 0.0, 0.0, -relativeHeight};
    success = A.relativeMoveBoth(relativePose, relativePose);

    /* ******************** 31 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 31 ~ moving both hands forward");

    relativePose = {0.0, 0.0, 0.0, 0.3, 0.0, 0.0};
    success = A.relativeMoveBoth(relativePose, relativePose);

    /* ******************** 32 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 32 ~ moving both hands up and backward");

    relativePose = {0.0, 0.0, 0.0, -0.15, 0.0, 0.1};
    success = A.relativeMoveBoth(relativePose, relativePose);

    /* ******************** 33 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 33 ~ opening both grippers");

    G.openGripper("R");
    G.openGripper("L");
    ros::Duration(2.0).sleep();

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
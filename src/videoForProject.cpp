#include <talos_clothes_wbc/main.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "videoForProject");
    ros::NodeHandle node_handle;

    grippers G(node_handle);
    headMove H(node_handle);
    armsMove A(node_handle);
    torsoMove T(node_handle);
    upperBodyMove UB(A, T);

    // ros::Publisher point_cloud_trigger_pub = node_handle.advertise<std_msgs::Int32>("/PCrequest", 1, true);
    ros::Publisher point_cloud_trigger_pub = node_handle.advertise<std_msgs::Int32>("/PCrequest", 1, false);
    ros::Publisher data_recorder_trigger_pub = node_handle.advertise<std_msgs::Bool>("/data_recorder/trigger", 1, false);
    ros::Publisher video_trigger_pub = node_handle.advertise<std_msgs::Empty>("/record_trigger", 1);
    // ros::Publisher cloth_defect_trigger_pub = node_handle.advertise<std_msgs::Bool>("/quality_control/trigger", 1, false);

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

    G.openGripper("R");
    G.openGripper("L");
    ros::Duration(2.0).sleep();

    /* ******************************************* */
    std::cout << "\nPress 1 to start the program: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************* tmp ******************* */
    H.qualityControlMove(headDuration);
    /* ******************************************* */

    /* ******************** 2 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 2 ~ moving torso right");

    if (!UB.captureReferenceYaw())
    {
        return 0;
    }
    UB.rotateTorsoWithArms(-0.5, 4.0);

    /* ******************** 3 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 3 ~ moving head down");

    success = H.jointsMove(headJointsDown, headDuration);
    std::cout << "success: " << success << std::endl;
    if (!success)
    {
        return 0;
    }

    ros::Duration(2.0).sleep();

    /* ******************** 4 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 4 ~ request PC analysis (height of table)");

    msgInt32.data = 3;
    point_cloud_trigger_pub.publish(msgInt32);

    /* ******************** 5 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 5 ~ reading height of table");

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
    ROS_INFO("\nSTEP 6 ~ request PC analysis");

    msgInt32.data = 1;
    point_cloud_trigger_pub.publish(msgInt32);

    /* ******************** 7 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 7 ~ reading highest point of point cloud");

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

    /* ******************** 8 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 8 ~ moving right arm");

    constexpr double DEG2RAD = M_PI / 180.0;

    const tf2::Vector3 target1(maxPoint[0], maxPoint[1], maxPoint[2]);

    if (!UB.poseRelativeToTorso({0.5, 0.5, -0.5, 0.5}, target1, tf2::Vector3(-0.1, 0.0, 0.3), poseR))
    {
        return 0;
    }
    success = A.absoluteMoveR(poseR, true);

    if (!UB.poseRelativeToTorso({0.0, 0.0, std::sin(45 * DEG2RAD), -std::sin(45 * DEG2RAD)}, target1, tf2::Vector3(0.0, 0.0, 0.1), poseR))
    {
        return 0;
    }
    success = A.absoluteMoveR(poseR, true);
    if (!success)
    {
        return 0;
    }

    /* ******************** 9 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 9 ~ moving right arm down");

    if (!UB.poseRelativeToTorso({0.0, 0.0, std::sin(45 * DEG2RAD), -std::sin(45 * DEG2RAD)}, target1, tf2::Vector3(0.0, 0.0, 0.0), poseR))
    {
        return 0;
    }
    success = A.absoluteMoveR(poseR, true);

    if (!success)
    {
        return 0;
    }

    ros::Duration(2.0).sleep();

    /* ******************** 10 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 10 ~ closing right gripper");

    G.closeGripper("R", 1);
    ros::Duration(2.0).sleep();

    /* ******************** 11 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 11 ~ moving head up");

    H.jointsMove(headJointsUp, headDuration);
    if (!success)
    {
        return 0;
    }

    /* ******************** 12 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 12 ~ moving right arm up");

    // poseR = {0.5, 0.5, -0.5, 0.5, 0.5, -1 * gripperLength, 0.6}; // TODO
    // success = A.absoluteMoveR(poseR, true);

    // if (!success)
    // {
    //     return 0;
    // }

    geometry_msgs::PoseStamped pivotPose;
    if (!A.getPoseFromTF("odom", "torso_1_link", pivotPose))
    {
        return 0;
    }
    const tf2::Vector3 pivot(pivotPose.pose.position.x, pivotPose.pose.position.y, pivotPose.pose.position.z);

    double yaw; // about -0.5 now
    if (!UB.getCurrentYaw(yaw))
    {
        return 0;
    }

    const tf2::Vector3 finalPos(0.5, -gripperLength, 0.6); // pose wanted AFTER the torso is back
    poseR = UB.poseRelativeToTorso({0.5, 0.5, -0.5, 0.5}, pivot, finalPos - pivot, yaw);
    success = A.absoluteMoveR(poseR, true);

    /* ******************** 13 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 13 ~ moving torso left");

    // if (!UB.captureReferenceYaw())
    // {
    //     return 0;
    // }
    // UB.rotateTorsoWithArms(0.5, 4.0);

    if (!UB.getCurrentYaw(yaw))
    {
        return 0;
    }
    if (!UB.rotateTorsoWithArms(-yaw, 4.0))
    {
        ROS_ERROR("Torso return failed");
        return 0;
    }

    /* ******************** 14 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 14 ~ moving head down");

    success = H.jointsMove(headJointsDown, headDuration);
    if (!success)
    {
        return 0;
    }
    ros::Duration(headDuration).sleep();

    ros::Duration(2.0).sleep();

    /* ******************** 15 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 15 ~ request PC analysis");

    msgInt32.data = 2;
    point_cloud_trigger_pub.publish(msgInt32);
    
    /* ******************** 16 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 16 ~ reading lowest point of point cloud");

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

            // target point for left
            minPoint[1] = minPoint[1] + gripperLength + 0.05;

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
    
    /* ******************** 17 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 17 ~ moving left arm");

    geometry_msgs::PoseStamped current_pose;
    A.getLeftGripperPose(current_pose);
    geometry_msgs::Point position = current_pose.pose.position;
    poseL = {0.5, 0.5, 0.5, -0.5, minPoint[0], position.y, position.z};
    success = A.absoluteMoveL(poseL, true);

    if (!success)
    {
        return 0;
    }

    /* ******************** 18 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 18 ~ moving left arm to the point");

    poseL = {0.5, 0.5, 0.5, -0.5, minPoint[0], minPoint[1], minPoint[2]};
    success = A.absoluteMoveL(poseL, true);

    if (!success)
    {
        return 0;
    }
    
    /* ******************** 19 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 19 ~ closing left gripper");

    G.closeGripper("L", 2);
    ros::Duration(2.0).sleep();

    /* ******************** 20 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 20 ~ moving left arm");

    poseL = {0.5, 0.5, 0.5, -0.5, 0.5, 0.35, minPoint[2] + 0.2};
    success = A.absoluteMoveL(poseL);

    if (!success)
    {
        return 0;
    }

    ros::Duration(2.0).sleep();

    /* ******************** 21 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 21 ~ opening right gripper");

    G.openGripper("R");
    ros::Duration(2.0).sleep();
    
    /* ******************** 22 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 22 ~ moving right arm");

    // std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.4, 0.4};
    poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.3, 0.4};
    success = A.absoluteMoveR(poseR, true);

    if (!success)
    {
        return 0;
    }

    poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.3, 0.2};
    success = A.absoluteMoveR(poseR, true);

    if (!success)
    {
        return 0;
    }

    /* ******************** 23 ******************* */ // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    std::cout << std::endl;
    ROS_INFO("\nSTEP 23 ~ moving left arm");

    poseL = {0.5, 0.5, 0.5, -0.5, 0.5, 0.1, 0.7};
    success = A.absoluteMoveL(poseL, true);

    if (!success)
    {
        return 0;
    }

    /* ******************** 24 ******************* */
    // std::cout << std::endl;
    // ROS_INFO("\nSTEP 24 ~ moving head down");

    // success = H.jointsMove(headJointsDown_rs, headDuration);
    // if (!success)
    // {
    //     return 0;
    // }
    // ros::Duration(headDuration).sleep();

    /* ******************** 25 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 25 ~ CeDiRNet");

    H.cedirnetMove(headDuration);
    
    /* ******************** 26 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 26 ~ CeDiRNet");

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

    /* ******************** 27 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 27 ~ moving right arm");

    poseR = {0.5, 0.5, -0.5, 0.5, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
    success = A.absoluteMoveR(poseR, true);

    if (!success)
    {
        return 0;
    }
    
    /* ******************** 28 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 28 ~ closing right gripper");

    G.closeGripper("R", 2);
    ros::Duration(2.0).sleep();

    /* ******************** 29 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 29 ~ moving left arm");

    poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.2, 0.6};
    success = A.absoluteMoveL(poseL, false);
    
    if (!success)
    {
        return 0;
    }

    ros::Duration(4.0).sleep();

    /* ******************** 30 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 30 ~ moving right arm");

    poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.2, 0.6};
    success = A.absoluteMoveR(poseR, true);
    
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
    
    /* ******************** 31 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 31 ~ moving both arms (by force)");

    A.forceMove_old(15);

    /* ******************** 32 ******************* */
    // TODO: cloth defect
    // std::cout << std::endl;
    // ROS_INFO("\nSTEP 32 ~ requesting cloth defect detection");

    // msgBool.data = true;
    // cloth_defect_trigger_pub.publish(msgBool);

    /* ******************** 33 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 33 ~ moving both hands down");

    // relativePose = {0.0, 0.0, 0.0, 0.0, 0.0, -relativeHeight};
    relativePose = {0.0, 0.0, 0.0, 0.0, 0.0, -relativeHeight+0.05};
    success = A.relativeMoveBoth(relativePose, relativePose);

    /* ******************** 34 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 34 ~ moving both hands forward");

    relativePose = {0.0, 0.0, 0.0, 0.3, 0.0, 0.0};
    success = A.relativeMoveBoth(relativePose, relativePose);

    /* ******************** 35 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 35 ~ moving both hands up and backward");

    relativePose = {0.0, 0.0, 0.0, -0.15, 0.0, 0.1};
    success = A.relativeMoveBoth(relativePose, relativePose);

    /* ******************** 36 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 36 ~ opening both grippers");

    G.openGripper("R");
    G.openGripper("L");
    ros::Duration(2.0).sleep();







    /* ******************************************* */
    std::cout << "\nPress 1 to finish the program: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    G.openGripper("R");
    G.openGripper("L");
    ros::Duration(2.0).sleep();

    poseR = {0.5, 0.5, -0.5, 0.5, 0.3, -0.4, 0.3};
    success = A.absoluteMoveR(poseR, false);
    if (!success)
    {
        return 0;
    }

    ros::Duration(4.0).sleep();

    poseL = {0.5, 0.5, 0.5, -0.5, 0.3, 0.4, 0.3};
    success = A.absoluteMoveL(poseL, true);
    if (!success)
    {
        return 0;
    }

}

/*
0) rostopic pub /whole_body_kinematic_controller/reference_ref sensor_msgs/JointState "{name: ['head_1_joint', 'head_2_joint', 'torso_1_joint', 'torso_2_joint'], position: [0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0], effort: [0.0, 0.0, 0.0, 0.0]}"

5) rostopic pub /heightOfTable std_msgs/Float32 "data: -0.16"

7) rostopic pub /highPCpoint std_msgs/Float32MultiArray "{data: [0.45, -0.45, 0.1]}"

15) rostopic pub /lowPCpoint std_msgs/Float32MultiArray "{data: [0.4, 0.1, 0.35]}"
*/
#include <iostream>
#include <typeinfo>
#include <ros/ros.h>

#include <talos_clothes_wbc/arms_move.h>


void printVector(const std::vector<double>& vec)
{
    for (size_t i = 0; i < vec.size(); i++)
    {
        std::cout << vec[i] << " ";
    }
    std::cout << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle node_handle;

    armsMove A(node_handle);

    std::vector<double> jointsR = A.getJointValuesR();
    std::vector<double> jointsL = A.getJointValuesL();
    std::cout << "==== right arm joints: ";
    printVector(jointsR);
    std::cout << "==== left arm joints: ";
    printVector(jointsL);

    int x;

    /*std::cout << "Press 1 to move right arm: ";
    std::cin >> x;

    if (x == 1)
    {
        // A.absoluteMoveR({0.5, 0.5, -0.5, 0.5, 0.5, 0.0, 0.7});
        // A.absoluteMoveR({0.464952165761, -0.671610383119, -0.506501710359, 0.27607063271, 0.511423027036, -0.171149201816, 1.11925415104}, true);
        A.absoluteMoveR({0.464952165761, -0.671610383119, -0.506501710359, 0.27607063271, 0.5, 0.0, 0.7}, true); // ==== right arm joints: 1.34924 -2.00189 0.170756 -0.543929 -0.50632 0.299494 0.675468 
        // A.absoluteMoveR({0.380065031053, -0.397813903587, -0.600928589099, 0.579809883571, 0.168287249365, -0.000214036536375, 1.39948828622}, true);
        // A.absoluteMoveR({0.380065031053, -0.397813903587, -0.600928589099, 0.579809883571, 0.5, 0.0, 0.7}, true);
        geometry_msgs::PoseStamped current_pose;
        A.getRightGripperPose(current_pose);
        std::vector<double> jointsR = A.getJointValuesR();
        std::cout << "==== right arm joints: ";
        printVector(jointsR);
    }*/

    bool success;

    ROS_INFO("\nMoving both arms");
    std::cout << "Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.2, 0.6};
        success = A.absoluteMoveR(poseR);
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.2, 0.6};
        success = A.absoluteMoveL(poseL);
    }
    else if (x != 2)
    {
        return 0;
    }

    std::cout << "Press 1 to print grippers poses or 2 to skip: ";
    std::cin >> x;

    if (x == 1)
    {
        geometry_msgs::PoseStamped rightPose;
        geometry_msgs::PoseStamped leftPose;
        A.getRightGripperPose(rightPose);
        A.getLeftGripperPose(leftPose);
        std::cout << "RIGHT ARM: " << rightPose.pose.position.x << ", " << rightPose.pose.position.y << ", " << rightPose.pose.position.z << std::endl;
        std::cout << "LEFT ARM: " << leftPose.pose.position.x << ", " << leftPose.pose.position.y << ", " << leftPose.pose.position.z << std::endl;
    }
    else if (x != 2)
    {
        return 0;
    }

    /*ROS_INFO("\nMoving right arm");
    std::cout << "Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        A.relativeMoveR({0.0, 0.0, 0.0, 0.0, -0.1, 0.0});
    }
    else if (x != 2)
    {
        return 0;
    }*/

    /*ROS_INFO("\nMoving left arm");
    std::cout << "Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        A.relativeMoveL({0.0, 0.0, 0.0, 0.0, 0.1, 0.0});
    }
    else if (x != 2)
    {
        return 0;
    }*/

    std::cout << std::endl;
    ROS_INFO("\nMoving both arms (by force)");
    std::cout << "Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        A.forceMove_old(15);
    }
    else if (x != 2)
    {
        return 0;
    }

    std::cout << std::endl;
    ROS_INFO("\nMoving both arms up");
    std::cout << "Press 1 to execute command or 2 to skip this command: ";
    std::cin >> x;

    if (x == 1)
    {
        geometry_msgs::PoseStamped rightPose;
        geometry_msgs::PoseStamped leftPose;
        A.getRightGripperPose(rightPose);
        A.getLeftGripperPose(leftPose);
        std::cout << "RIGHT ARM: " << rightPose.pose.position.x << ", " << rightPose.pose.position.y << ", " << rightPose.pose.position.z << std::endl;
        std::cout << "LEFT ARM: " << leftPose.pose.position.x << ", " << leftPose.pose.position.y << ", " << leftPose.pose.position.z << std::endl;
        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, rightPose.pose.position.x, rightPose.pose.position.y, 0.6};
        success = A.absoluteMoveR(poseR);
        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, leftPose.pose.position.x, leftPose.pose.position.y, 0.6};
        success = A.absoluteMoveL(poseL);
    }
    else if (x != 2)
    {
        return 0;
    }

    std::cout << "Press 1 to print grippers poses or 2 to skip: ";
    std::cin >> x;

    if (x == 1)
    {
        geometry_msgs::PoseStamped rightPose;
        geometry_msgs::PoseStamped leftPose;
        A.getRightGripperPose(rightPose);
        A.getLeftGripperPose(leftPose);
        std::cout << "RIGHT ARM: " << rightPose.pose.position.x << ", " << rightPose.pose.position.y << ", " << rightPose.pose.position.z << std::endl;
        std::cout << "LEFT ARM: " << leftPose.pose.position.x << ", " << leftPose.pose.position.y << ", " << leftPose.pose.position.z << std::endl;
    }
    else if (x != 2)
    {
        return 0;
    }
}


/*
rostopic pub -r 100 /right_wrist_ft geometry_msgs/WrenchStamped "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'wrist_right_ft_link'}, wrench: {force: {x: 0.0, y: 0.0, z: 20.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"

rostopic pub -r 100 /left_wrist_ft geometry_msgs/WrenchStamped "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'wrist_left_ft_link'}, wrench: {force: {x: 0.0, y: 0.0, z: 20.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"
*/

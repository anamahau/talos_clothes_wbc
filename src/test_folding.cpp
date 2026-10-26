#include <talos_clothes_wbc/main.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_folding_node");
    ros::NodeHandle node_handle;

    grippers G(node_handle);
    headMove H(node_handle);
    armsMove A(node_handle);

    ros::Publisher point_cloud_trigger_pub = node_handle.advertise<std_msgs::Int32>("/PCrequest", 1, false);

    /* ******************************************* */
    std::cout << "\nPress 1 to move arms to home pose or 2 to skip: ";
    std::cin >> x;
    if (x != 1 and x != 2)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 1 ******************** */
    std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.3, -0.4, 0.3};
    std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.3, 0.4, 0.3};

    if (x == 1)
    {
        std::cout << std::endl;
        ROS_INFO("\nSTEP 1 ~ move both hands to home pose");

        success = A.absoluteMoveR(poseR, false);
        if (!success)
        {
            return 0;
        }

        ros::Duration(4.0).sleep();

        success = A.absoluteMoveL(poseL, true);
        if (!success)
        {
            return 0;
        }
    }

    G.openGripper("R");
    G.openGripper("L");
    ros::Duration(2.0).sleep();

    /* ******************************************* */
    std::cout << "\nPress 1 to check the table height: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }

    // rostopic pub /heightOfTable std_msgs/Float32 "data: -0.16"
    /* ******************************************* */

    success = H.jointsMove(headJointsDown, headDuration);

    msgInt32.data = 3;
    point_cloud_trigger_pub.publish(msgInt32);

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

    success = H.jointsMove(headJointsUp, headDuration);

    /* ******************************************* */
    std::cout << "\nPress 1 to move arms to start pose: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 2 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 2 ~ move both hands to start pose");

    poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.25, 0.6};
    success = A.absoluteMoveL(poseL, false);
    if (!success)
    {
        return 0;
    }

    ros::Duration(4.0).sleep();

    poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.25, 0.6};
    success = A.absoluteMoveR(poseR, true);
    if (!success)
    {
        return 0;
    }

    /* ******************************************* */
    std::cout << "\nPress 1 to close both grippers: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    ros::Duration(3.0).sleep();
    G.closeGripper("L", 2);
    ros::Duration(3.0).sleep();
    G.closeGripper("R", 2);

    int y = 1;
    while (y == 1)
    {
        std::cout << "\nPress 1 to move both hands: ";
        std::cin >> x;
        if (x == 1)
        {
            A.relativeMoveR({0.0, 0.0, 0.0, 0.0, -0.05, 0.0});
            A.relativeMoveL({0.0, 0.0, 0.0, 0.0, 0.05, 0.0});
        }
        else
        {
            y = 0;
        }
    }

    /* ******************************************* */
    float relativeHeight;
    relativeHeight = (0.6 - tableHeight) - 0.20;
    std::cout << "relative move on z: " << relativeHeight << std::endl;
    std::cout << "\nPress 1 to continue (D): ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 33 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 33 ~ moving both hands down");

    // relativePose = {0.0, 0.0, 0.0, 0.0, 0.0, -relativeHeight};
    // relativePose = {0.0, 0.0, 0.0, 0.0, 0.0, -relativeHeight+0.05};
    relativePose = {0.0, 0.0, 0.0, 0.0, 0.0, -relativeHeight+0.1};
    success = A.relativeMoveBoth(relativePose, relativePose);

    /* ******************************************* */
    std::cout << "\nPress 1 to continue (F): ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 34 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 34 ~ moving both hands forward");

    relativePose = {0.0, 0.0, 0.0, 0.2, 0.0, 0.0};
    success = A.relativeMoveBoth(relativePose, relativePose);

    /* ******************************************* */
    std::cout << "\nPress 1 to continue (U and B): ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 35 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 35 ~ moving both hands up and backward");

    relativePose = {0.0, 0.0, 0.0, -0.1, 0.0, 0.1};
    success = A.relativeMoveBoth(relativePose, relativePose);

    /* ******************************************* */
    std::cout << "\nPress 1 to open both grippers: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 36 ******************* */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 36 ~ opening both grippers");

    G.openGripper("R");
    G.openGripper("L");
    ros::Duration(2.0).sleep();
}
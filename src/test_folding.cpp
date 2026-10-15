#include <talos_clothes_wbc/main.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_folding_node");
    ros::NodeHandle node_handle;

    grippers G(node_handle);
    headMove H(node_handle);
    armsMove A(node_handle);

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
    ROS_INFO("\nSTEP 1 ~ move both hands to home pose");

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
}
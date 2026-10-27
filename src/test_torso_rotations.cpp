#include <talos_clothes_wbc/main.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_torso_node");
    ros::NodeHandle node_handle;

    grippers G(node_handle);
    headMove H(node_handle);
    armsMove A(node_handle);
    torsoMove T(node_handle);
    upperBodyMove UB(A, T);

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
    std::cout << "\nPress 1 to move torso right: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    double fi = 0.5;
    double yaw;

    /* ******************** 2 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 2 ~ moving torso right");

    if (!UB.captureReferenceYaw())
    {
        return 0;
    }
    if (!UB.getCurrentYaw(yaw))
    {
        return 0;
    }
    std::cout << "Current yaw: " << yaw << ", desired rotation: " << -fi << std::endl;
    UB.rotateTorsoWithArms(-fi, 4.0);

    /* ******************************************* */
    std::cout << "\nPress 1 to move torso left: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 3 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 3 ~ moving torso right");

    if (!UB.captureReferenceYaw())
    {
        return 0;
    }
    if (!UB.getCurrentYaw(yaw))
    {
        return 0;
    }
    std::cout << "Current yaw: " << yaw << ", desired rotation: 0.0" << std::endl;
    UB.rotateTorsoWithArms(fi, 4.0);

    /* ******************************************* */
    std::cout << "\nPress 1 to move torso left: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 4 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 4 ~ moving torso right");

    if (!UB.captureReferenceYaw())
    {
        return 0;
    }
    if (!UB.getCurrentYaw(yaw))
    {
        return 0;
    }
    std::cout << "Current yaw: " << yaw << ", desired rotation: " << fi << std::endl;
    UB.rotateTorsoWithArms(fi, 4.0);

    /* ******************************************* */
    std::cout << "\nPress 1 to move torso right: ";
    std::cin >> x;
    if (x != 1)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************** 5 ******************** */
    std::cout << std::endl;
    ROS_INFO("\nSTEP 5 ~ moving torso right");

    if (!UB.captureReferenceYaw())
    {
        return 0;
    }
    if (!UB.getCurrentYaw(yaw))
    {
        return 0;
    }
    std::cout << "Current yaw: " << yaw << ", desired rotation: 0.0" << std::endl;
    UB.rotateTorsoWithArms(-fi, 4.0);

}
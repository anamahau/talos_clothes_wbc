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

    std::cout << "Press 1 to move right arm: ";
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
    }
}

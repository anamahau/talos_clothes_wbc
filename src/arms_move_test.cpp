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
    ros::init(argc, argv, "arms_move_test");
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
        A.relativeMoveR({0.0, 0.0, 0.0, 0.0, 0.0, -0.1});
    }
}
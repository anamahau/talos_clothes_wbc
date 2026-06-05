#include <talos_clothes_wbc/arms_move.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "armLoop");
    ros::NodeHandle node_handle;

    armsMove A(node_handle);

    std::cout << "Enter:\n";
    std::cout << "x y z hand(L/R)\n";
    std::cout << "Type q to quit\n\n";

    while (true)
    {
        double x, y, z;
        char hand;

        std::cout << "Input: ";

        std::string firstInput;
        std::cin >> firstInput;

        if (firstInput == "q" || firstInput == "Q")
            break;

        try
        {
            x = std::stod(firstInput);
        }
        catch (...)
        {
            std::cout << "Invalid input.\n";
            continue;
        }

        std::cin >> y >> z >> hand;

        bool success = false;

        if (hand == 'R' || hand == 'r')
        {
            std::vector<double> pose = {
                0.5, 0.5, -0.5, 0.5,
                x, y, z
            };
            success = A.absoluteMoveR(pose);
        }
        else if (hand == 'L' || hand == 'l')
        {
            std::vector<double> pose = {
                0.5, 0.5, 0.5, -0.5,
                x, y, z
            };
            success = A.absoluteMoveL(pose);
        }
        else
        {
            std::cout << "Hand must be L or R.\n";
            continue;
        }
    }

    return 0;
}
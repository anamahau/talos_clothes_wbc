#include <talos_clothes_wbc/arms_move.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "armLoop");
    ros::NodeHandle node_handle;

    armsMove A(node_handle);

    std::cout << "Enter:\n";
    std::cout << "x y z hand(L/R) orientation\n";
    std::cout << "orientation 1: grab from table, 2: regrab" << std::endl;
    std::cout << "Type q to quit\n\n";

    constexpr double DEG2RAD = M_PI / 180.0;

    while (true)
    {
        double x, y, z;
        char hand;
        int orientation;

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

        std::cin >> y >> z >> hand >> orientation;

        bool success = false;

        if (hand == 'R' || hand == 'r')
        {
            std::vector<double> pose;
            if (orientation == 1)
            {
                pose = {
                    0, 0, std::sin(45 * DEG2RAD), (-1) * std::sin(45 * DEG2RAD),
                    x, y, z
                };
            }
            else if (orientation == 2)
            {
                pose = {
                    0.5, 0.5, -0.5, 0.5,
                    x, y, z
                };
            }
            else
            {
                std::cout << "Orientation must be 1 or 2.\n";
                continue;
            }
            success = A.absoluteMoveR(pose);
        }
        else if (hand == 'L' || hand == 'l')
        {
            std::vector<double> pose;
            if (orientation == 1)
            {
                pose = {
                    0, 0, std::sin(45 * DEG2RAD), (-1) * std::sin(45 * DEG2RAD),
                    x, y, z
                };
            }
            else if (orientation == 2)
            {
                pose = {
                    0.5, 0.5, 0.5, -0.5,
                    x, y, z
                };
            }
            else
            {
                std::cout << "Orientation must be 1 or 2.\n";
                continue;
            }
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
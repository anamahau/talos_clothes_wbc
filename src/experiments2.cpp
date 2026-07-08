#include <talos_clothes_wbc/main.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "experiments2_node");

    ros::NodeHandle node_handle;

    grippers G(node_handle);
    headMove H(node_handle);
    armsMove A(node_handle);

    // ros::Publisher point_cloud_trigger_pub = node_handle.advertise<std_msgs::Int32>("/PCrequest", 1, false);
    ros::Publisher data_recorder_trigger_pub = node_handle.advertise<std_msgs::Bool>("/data_recorder/trigger", 1, false);
    ros::Publisher video_trigger_pub = node_handle.advertise<std_msgs::Empty>("/record_trigger", 1);

    std::vector<double> poseR = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> poseL = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    /* ******************************************* */
    video_trigger_pub.publish(std_msgs::Empty()); // pause
    std::cout << "\nPress 1 to move arms to home pose or 2 to skip: ";
    std::cin >> x;
    if (x == 1)
    {
        /* ******************** 1 ******************** */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 1 ~ move both hands");

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
    else if (x != 2)
    {
        return 0;
    }
    /* ******************************************* */

    /* ******************************************* */
    H.jointsMove(headJointsUp, headDuration);

    G.openGripper("R");
    G.openGripper("L");
    ros::Duration(2.0).sleep();
    /* ******************************************* */

    bool firstGrasp = true;

    int y = 1;
    while (y == 1)
    {
        /* ******************************************* */
        std::cout << "\nPress 1 to close left gripper: ";
        std::cin >> x;
        if (x == 1)
        {
            G.closeGripper("L", 2);
        }
        else
        {
            return 0;
        }
        /* ******************************************* */

        /* ******************************************* */
        if (firstGrasp)
        {
            firstGrasp = false;
            std::cout << "Start recording and press 1 to start the iteration: ";
            std::cin >> x;
        }
        else
        {
            std::cout << "\nPress 1 to start the iteration: ";
            std::cin >> x;
            video_trigger_pub.publish(std_msgs::Empty()); // resume
        }
        
        if (x != 1)
        {
            return 0;
        }
        /* ******************************************* */

        y = 0;

        /* ******************** 20 ******************* */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 20 ~ moving right arm");

        poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.4, 0.2};
        success = A.absoluteMoveR(poseR, true);

        if (!success)
        {
            return 0;
        }
        
        /* ******************** 21 ******************* */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 21 ~ moving left arm");

        poseL = {0.5, 0.5, 0.5, -0.5, 0.5, 0.1, 0.7};
        success = A.absoluteMoveL(poseL, true);

        if (!success)
        {
            return 0;
        }

        /* ******************** 23 ******************* */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 23 ~ CeDiRNet");

        H.cedirnetMove(headDuration);

        /* ******************** 24 ******************* */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 24 ~ CeDiRNet");
        std::cout << "\t  rostopic pub /cedirnet/goal_pose geometry_msgs/PoseStamped \"{pose: {position: {x: , y: , z: }, orientation: {x: 0.5, y: 0.5, z: -0.5, w: 0.5}}}\"\n";

        video_trigger_pub.publish(std_msgs::Empty()); // pause

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
        video_trigger_pub.publish(std_msgs::Empty()); // resume
        if (x != 1)
        {
            return 0;
        }
        /* ******************************************* */

        /* ******************** 25 ******************* */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 25 ~ moving right arm");

        poseR = {0.5, 0.5, -0.5, 0.5, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y-0.15, msgCedirnet->pose.position.z};
        // std::vector<double> poseR = {msgCedirnet->pose.orientation.x, msgCedirnet->pose.orientation.y, msgCedirnet->pose.orientation.z, msgCedirnet->pose.orientation.w, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
        success = A.absoluteMoveR(poseR, true);

        if (!success)
        {
            return 0;
        }
        
        /* ******************** 26 ******************* */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 26 ~ closing right gripper");

        G.closeGripper("R", 2);

        ros::Duration(2.0).sleep();

        /* ******************** 27 ******************* */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 27 ~ moving left arm");

        poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.2, 0.6};
        success = A.absoluteMoveL(poseL, false);
        
        if (!success)
        {
            return 0;
        }

        ros::Duration(4.0).sleep();

        /* ******************** 28 ******************* */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 28 ~ moving right arm");

        poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.2, 0.6};
        success = A.absoluteMoveR(poseR, true);
        
        if (!success)
        {
            return 0;
        }

        video_trigger_pub.publish(std_msgs::Empty()); // pause

        /* ******************************************* */
        std::cout << "\nPress 1 to continue the program: ";
        std::cin >> x;
        video_trigger_pub.publish(std_msgs::Empty()); // resume
        if (x != 1)
        {
            return 0;
        }
        /* ******************************************* */

        /* ******************** 29 ******************* */
        std::cout << std::endl;
        ROS_INFO("\nSTEP 29 ~ moving both arms (by force)");

        A.forceMove_old(15);
    
        video_trigger_pub.publish(std_msgs::Empty()); // pause
        std::cout << "\nPress 1 to repeat the CeDiRNet grasp: ";
        std::cin >> y;

        if (y == 1)
        {
            G.openGripper("R");
            G.openGripper("L");
            ros::Duration(2.0).sleep();
        }
    }
}
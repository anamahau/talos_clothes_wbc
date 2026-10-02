#include <talos_clothes_wbc/main.h>

#include <regex>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace fs = boost::filesystem;


int getNextImageIndex(const std::string& dir, const std::string& prefix)
{
    if (!fs::exists(dir))
    {
        fs::create_directories(dir);
        return 0;
    }

    int maxIndex = -1;
    std::regex pattern(prefix + R"(_(\d{6})\.png)");

    for (const auto& entry : fs::directory_iterator(dir))
    {
        std::string filename = entry.path().filename().string();
        std::smatch match;
        if (std::regex_match(filename, match, pattern))
        {
            int idx = std::stoi(match[1].str());
            maxIndex = std::max(maxIndex, idx);
        }
    }

    return maxIndex + 1;
}

bool saveImageFromTopic(ros::NodeHandle& nh,
                         const std::string& topic,
                         const std::string& dir,
                         const std::string& prefix = "image",
                         double timeout = 5.0)
{
    sensor_msgs::ImageConstPtr msg = ros::topic::waitForMessage<sensor_msgs::Image>(
        topic, nh, ros::Duration(timeout)
    );

    if (!msg)
    {
        ROS_ERROR_STREAM("No image received on topic: " << topic);
        return false;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return false;
    }

    int index = getNextImageIndex(dir, prefix);

    char buf[16];
    snprintf(buf, sizeof(buf), "%06d", index);
    std::string filepath = dir + "/" + prefix + "_" + buf + ".png";

    if (!cv::imwrite(filepath, cv_ptr->image))
    {
        ROS_ERROR_STREAM("Failed to write image to: " << filepath);
        return false;
    }

    ROS_INFO_STREAM("Saved image to: " << filepath);
    return true;
}


struct Quaternion
{
    double w;
    double x;
    double y;
    double z;
};

Quaternion EulerToQuaternion(double roll, double pitch, double yaw)
{
    // Abbreviations for the various angular functions
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    Quaternion q;

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "experiments2_node");

    // Quaternion q = EulerToQuaternion(1.3628772478290372, 0.07224361352750375, -1.9667040032384886);
    // std::cout << "quaternion: " << q.x << ", " << q.y << ", " << q.z << ", " << q.w << std::endl;

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
            ros::Duration(2.0).sleep();
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
                std::cout << "     quaternion: " << msgCedirnet->pose.orientation.x << ", " << msgCedirnet->pose.orientation.y << ", " << msgCedirnet->pose.orientation.z << ", " << msgCedirnet->pose.orientation.w << std::endl;
                waitIdx = 4;
            }
        }
        if (!msgCedirnet)
        {
            return 0;
        }

        /* ******************************************* */
        int qs;
        std::cout << "\nPress 1 (CeDiRNet quaternion) or 2 (default quaternion): ";
        std::cin >> qs;
        if (qs == 1)
        {
            poseR = {msgCedirnet->pose.orientation.x, msgCedirnet->pose.orientation.y, msgCedirnet->pose.orientation.z, msgCedirnet->pose.orientation.w, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y-0.2, msgCedirnet->pose.position.z};
        }
        else if (qs == 2)
        {
            poseR = {0.5, 0.5, -0.5, 0.5, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y-0.2, msgCedirnet->pose.position.z};
        }
        else
        {
            return 0;
        }
        std::cout << "==== poseR: ";
        for (double a : poseR)
        {
            std::cout << a << " ";
        }
        std::cout << std::endl;
        /* ******************************************* */

        /* ******************************************* */
        std::cout << "\nPress 1 to continue the program or 2 to skip this iteration: ";
        std::cin >> x;
        if (x == 1)
        {
            video_trigger_pub.publish(std_msgs::Empty()); // resume

            /* ******************** 25 ******************* */
            std::cout << std::endl;
            ROS_INFO("\nSTEP 25 ~ moving right arm");

            // poseR = {0.5, 0.5, -0.5, 0.5, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y-0.2, msgCedirnet->pose.position.z};
            // poseR = {msgCedirnet->pose.orientation.x, msgCedirnet->pose.orientation.y, msgCedirnet->pose.orientation.z, msgCedirnet->pose.orientation.w, msgCedirnet->pose.position.x, msgCedirnet->pose.position.y, msgCedirnet->pose.position.z};
            success = A.absoluteMoveR(poseR, true);

            if (!success)
            {
                return 0;
            }
            
            /* ******************** 26 ******************* */
            std::cout << std::endl;
            ROS_INFO("\nSTEP 26 ~ closing right gripper");

            G.closeGripper("R", 3);

            ros::Duration(2.0).sleep();

            /* ******************** 27 ******************* */
            std::cout << std::endl;
            ROS_INFO("\nSTEP 27 ~ moving left arm");

            poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.25, 0.6};
            success = A.absoluteMoveL(poseL, false);
            
            if (!success)
            {
                return 0;
            }

            ros::Duration(4.0).sleep();

            /* ******************** 28 ******************* */
            std::cout << std::endl;
            ROS_INFO("\nSTEP 28 ~ moving right arm");

            poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.25, 0.6};
            success = A.absoluteMoveR(poseR, true);
            
            if (!success)
            {
                return 0;
            }

            video_trigger_pub.publish(std_msgs::Empty()); // pause

            /* ******************************************* */
            double forceR = A.computeForceNorm(A.right_ft_msg_);
            double forceL = A.computeForceNorm(A.left_ft_msg_);
            std::cout << "right force: " << forceR << ", left force: " << forceL << std::endl;
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

            A.forceMove_old(static_cast<int>(std::max(forceR, forceL)) + 3);
            // A.forceMove_old(25);

            ros::Duration(3.0).sleep();
        
            video_trigger_pub.publish(std_msgs::Empty()); // pause
        }
        else if (x != 2)
        {
            return 0;
        }

        saveImageFromTopic(node_handle, "/camera2/camera/color/image_raw", "/home/pal/docker_anamarija/evaluation");

        std::cout << "\nPress 1 to repeat the CeDiRNet grasp: ";
        std::cin >> y;

        // if (y == 1)
        // {
        //     G.openGripper("R");
        //     G.openGripper("L");
        //     ros::Duration(2.0).sleep();
        // }
        G.openGripper("R");
        G.openGripper("L");
        ros::Duration(2.0).sleep();
    }
}
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
                         const std::string& prefix = "reference_image",
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "experiments2_node");

    ros::NodeHandle node_handle;

    grippers G(node_handle);
    headMove H(node_handle);
    armsMove A(node_handle);
    
    std::cout << "\nPress 1 to move arms to start pose: ";
    std::cin >> x;
    if (x == 1)
    {
        G.openGripper("R");
        G.openGripper("L");

        std::cout << std::endl;
        ROS_INFO("\nSTEP 1 ~ move both hands");

        std::vector<double> poseL = {0.5, 0.5, 0.5, -0.5, 0.4, 0.25, 0.6};
        success = A.absoluteMoveL(poseL, false);
        if (!success)
        {
            return 0;
        }

        ros::Duration(4.0).sleep();

        std::vector<double> poseR = {0.5, 0.5, -0.5, 0.5, 0.4, -0.25, 0.6};
        success = A.absoluteMoveR(poseR, true);
        if (!success)
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }

    std::cout << "\nPress 1 to close both grippers: ";
    std::cin >> x;
    if (x == 1)
    {
        ros::Duration(2.0).sleep();
        G.closeGripper("L", 2);
        ros::Duration(2.0).sleep();
        G.closeGripper("R", 2);
    }
    else
    {
        return 0;
    }

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

    ros::Duration(2.0).sleep();
    saveImageFromTopic(node_handle, "/camera2/camera/color/image_raw", "/home/pal/docker_anamarija/evaluation");
}
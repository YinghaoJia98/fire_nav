#include <ros/ros.h>
#include <fire_nav/fire_nav.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fire_nav_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ROS_INFO("Hello world.");
    // NavManager NavManager_(nh, nh_private);
    FireManager FireManager_(nh, nh_private);
    int SpinnerThread_;
    nh.param<int>("/fire_nav/fire_nav_settings/SpinnerThread", SpinnerThread_, 1);
    ros::AsyncSpinner spinner(SpinnerThread_); // Use n threads
    spinner.start();
    ros::waitForShutdown();
    ROS_INFO("The fire_nav_node is closing, see you.");
    return 0;
}

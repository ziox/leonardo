#include <ros/ros.h>

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle handle("~");
    ros::spin();
    return 0;
}
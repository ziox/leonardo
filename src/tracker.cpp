#include <ros/ros.h>

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle handle("~");
    ros::spin();
    return 0;
}
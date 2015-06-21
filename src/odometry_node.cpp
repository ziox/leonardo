#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ardrone_autonomy/Navdata.h>
#include <cmath>


struct OdometryNode
{
    OdometryNode(ros::NodeHandle& nh)
        : zero_yaw(0.0)
        , actual_yaw(0.0)
        , previous_tm(-1)
        , odometry_topic(nh.advertise<geometry_msgs::TransformStamped>("odometry", 10))
        , navdata_topic(nh.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata", 10, &OdometryNode::onNavdata, this))
        , command_topic(nh.subscribe<std_msgs::String>("command", 10, &OdometryNode::onCommand, this))
    {}

    void run()
    {
        ros::spin();
    }

private:
    void onCommand(std_msgs::String msg)
    {
        std::string command = msg.data;
        if (command == "zero")
        {
            zero_yaw = actual_yaw;
            position = tf::Vector3(0, 0, 0);
        }
    }

    void onNavdata(ardrone_autonomy::Navdata navdata)
    {
        if (previous_tm < 0)
        {
            previous_tm = navdata.tm;
            return; // DO NOTHING!
        }

        // navdata.tm [us]
        double dt_us = navdata.tm - previous_tm;
        if (dt_us < 0)
        {
            ROS_ERROR("NEGATIVE TIME");
            previous_tm = navdata.tm;
            return;
        }

        double dt_ns = dt_us * 1.e3;

        // navdata.rotX [degrees]
        double roll = navdata.rotX / 180.0 * 3.14;
        // navdata.rotY [degrees]
        double pitch = navdata.rotY / 180.0 * 3.14;
        // navdata.rotZ [degrees]
        double yaw = navdata.rotZ / 180.0 * 3.14;
        actual_yaw = yaw;

        // navdata.vx [mm/s]
        double dx = (navdata.vx) * dt_ns / 1.e12;
        // navdata.vy [mm/s]
        double dy = (navdata.vy) * dt_ns / 1.e12;

        // navdata.altd [mm]
        double z = navdata.altd / 1.e3;

        orientation = tf::createQuaternionFromRPY(roll, pitch, yaw - zero_yaw);
        tf::Vector3 ds(dx, dy, 0.);
        ds = ds.rotate(tf::Vector3(0., 0., 1.), yaw - zero_yaw);
        position = tf::Vector3(position.x() + ds.x(), position.y() + ds.y(), z);
        timestamp.fromNSec(navdata.tm * 1e3);
        previous_tm = navdata.tm;

        publishOdometry();
    }

    void publishOdometry()
    {
        tf::Transform tf_transform(orientation, position);
        geometry_msgs::Transform transform;
        tf::transformTFToMsg(tf_transform, transform);

        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = timestamp;
        transform_stamped.transform = transform;

        odometry_topic.publish<geometry_msgs::TransformStamped>(transform_stamped);

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf_transform,
                ros::Time::now(),
                "/map",
                "/the_odometry_reference"));
    }

    double zero_yaw;

    // Drone State
    tf::Vector3 position;
    tf::Quaternion orientation;
    double actual_yaw;
    ros::Time timestamp;
    double previous_tm;

    ros::Subscriber navdata_topic;
    ros::Subscriber command_topic;
    ros::Publisher odometry_topic;
    tf::TransformBroadcaster broadcaster;
};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle handle("~");
    OdometryNode(handle).run();
    return 0;
}

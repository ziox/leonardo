#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "utils.h"
#include <csignal>
#include <string>
#include <cmath>


struct Marker
{
    Marker(double x, double y)
        : absolute_position(tf::createIdentityQuaternion(), tf::Vector3(x, y, 0.0))
        , used(true)
    {}

    tf::Transform absolute_position;
    tf::Transform tf_to_drone;

    unsigned int frames;
    double sigma;
    bool used;
};


struct LocalizationNode
{
    LocalizationNode(ros::NodeHandle& nh)
        : the_marker_428(0, 0)
        , the_marker_341(0, 0)
        , the_marker_985(2, 0)
        , odometry_topic(nh.subscribe<geometry_msgs::TransformStamped>("odometry", 10, &LocalizationNode::onOdometry, this))
        , markers_topic(nh.subscribe<geometry_msgs::TransformStamped>("/marker_detector/markers", 10, &LocalizationNode::onMarker, this))
        , estimate_topic(nh.advertise<geometry_msgs::TransformStamped>("odometry", 10))
    {
        the_markers = std::vector<Marker*> {&the_marker_428, &the_marker_341, &the_marker_985};

        // ArDrone OK
        tf_drone_to_camera = tf::Transform(tf::Quaternion(-0.7071, 0.7071, 0.0, 0.0), tf::Vector3(0.0, 0.0, 0.0));

        the_tf_odom = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0.0, 0.0, 0.0));
        the_tf_odom_to_drone = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0.0, 0.0, 0.0));
    }

    void run()
    {
        ros::Rate loop_rate(60); // Hz
        while (ros::ok())
        {
            the_tf_estimate = the_tf_odom * the_tf_odom_to_drone;

            for (auto marker : the_markers)
            {
                if (!marker->used)
                {
                    the_tf_odom = marker->absolute_position * marker->tf_to_drone * the_tf_odom_to_drone.inverse();
                    the_tf_estimate = marker->absolute_position * marker->tf_to_drone;
                    marker->used = true;
                }
            }

            publishEstimate();

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void onOdometry(geometry_msgs::TransformStamped odometry_msg)
    {
        tf::transformMsgToTF(odometry_msg.transform, the_tf_odom_to_drone);
    }

    void onMarker(geometry_msgs::TransformStamped camera_to_marker_msg)
    {
        Marker* marker = nullptr;
        if ("marker_428" == camera_to_marker_msg.child_frame_id)
        {
            marker = &the_marker_428;
        }
        else if ("marker_341" == camera_to_marker_msg.child_frame_id)
        {
            marker = &the_marker_341;
        }
        else if ("marker_985" == camera_to_marker_msg.child_frame_id)
        {
            marker = &the_marker_985;
        }

        if (!marker) return;

        tf::StampedTransform tf_camera_to_marker;
        tf::transformStampedMsgToTF(camera_to_marker_msg, tf_camera_to_marker);
        tf::Transform tf_marker_to_drone = (tf_drone_to_camera * tf_camera_to_marker).inverse();

        tf::Vector3 position = tf_marker_to_drone.getOrigin();
        tf::Quaternion orientation = tf_marker_to_drone.getRotation();

        if (marker->frames > 0)
        {
            position = (position + marker->tf_to_drone.getOrigin() * marker->frames) / (1. + marker->frames);
            orientation = marker->tf_to_drone.getRotation().slerp(orientation, 1.0 / marker->frames);
            marker->frames = marker->frames < 10 ? marker->frames + 1 : marker->frames;
        }
        else
        {
            marker->frames = 1;
        }

        marker->used = false;

        marker->tf_to_drone.setOrigin(position);
        marker->tf_to_drone.setRotation(orientation);
    }

    void publishEstimate()
    {
        geometry_msgs::Transform estimate;
        tf::transformTFToMsg(the_tf_estimate, estimate);

        geometry_msgs::TransformStamped estimate_stamped;
        estimate_stamped.header.stamp = ros::Time::now();
        estimate_stamped.transform = estimate;

        estimate_topic.publish<geometry_msgs::TransformStamped>(estimate_stamped);

        broadcaster.sendTransform(
            tf::StampedTransform(
                the_tf_estimate,
                ros::Time::now(),
                "/map",
                "/the_estimate"));
    }

    Marker the_marker_428;
    Marker the_marker_341;
    Marker the_marker_985;
    std::vector<Marker*> the_markers;

    tf::Transform the_tf_odom;
    tf::Transform the_tf_odom_to_drone;
    tf::Transform the_tf_estimate;
    tf::Transform tf_drone_to_camera;

    ros::Subscriber odometry_topic;
    ros::Subscriber markers_topic;
    ros::Publisher estimate_topic;
    tf::TransformBroadcaster broadcaster;
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "localization");
    ros::NodeHandle handle("~");
    LocalizationNode(handle).run();
    return 0;
}

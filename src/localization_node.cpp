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

/***
!!!!

Trasformazione di un punto da A a W

P_W = H_W->A * P_A

!!!!
***/

// Input
ros::Subscriber odometry_topic;
ros::Subscriber markers_topic;

// Output
ros::Publisher estimate_topic;

tf::Transform the_tf_odom;
tf::Transform the_tf_odom_to_drone;
tf::Transform the_tf_estimate;

tf::Transform tf_drone_to_camera;

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

//Marker the_marker_23;
// Marker the_marker_428(0, 0);
// Marker the_marker_341(0, 2.7);
// Marker the_marker_985(2, 0);
Marker the_marker_428(0, 0);
// Marker the_marker_341(0.7, 2.7);
Marker the_marker_341(0, 0);
Marker the_marker_985(2, 0);

auto the_markers = std::vector<Marker*> {&the_marker_428, &the_marker_341, &the_marker_985};

void process_odometry(geometry_msgs::TransformStamped odometry_msg)
{
    // tf::transformStampedMsgToTF(odometry_msg, the_tf_odom_to_drone);
    tf::transformMsgToTF(odometry_msg.transform, the_tf_odom_to_drone);
}

void process_marker(geometry_msgs::TransformStamped camera_to_marker_msg)
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
        ROS_ERROR("985!");
        marker = &the_marker_985;
    }

    if (!marker) return;

    tf::StampedTransform tf_camera_to_marker;
    tf::transformStampedMsgToTF(camera_to_marker_msg, tf_camera_to_marker);
    tf::Transform tf_marker_to_drone = (tf_drone_to_camera * tf_camera_to_marker).inverse();

    // marker->tf_to_drone = tf_marker_to_drone;

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

    // ROS_ERROR("the_marker_23 = (%f, %f, %f)",
    //         marker->position.m_floats[0],
    //         marker->position.m_floats[1],
    //         marker->position.m_floats[2]);

    // static tf::TransformBroadcaster broadcaster;
    // broadcaster.sendTransform(
    //     tf::StampedTransform(
    //         tf::Transform(marker->orientation , marker->position),
    //         marker->time,
    //         "/map",
    //         "/the_marker_23_drone"));
}

void publish_estimate()
{
    geometry_msgs::Transform estimate;
    tf::transformTFToMsg(the_tf_estimate, estimate);

    geometry_msgs::TransformStamped estimate_stamped;
    estimate_stamped.header.stamp = ros::Time::now();
    estimate_stamped.transform = estimate;

    estimate_topic.publish<geometry_msgs::TransformStamped>(estimate_stamped);

    // ROS_ERROR("the_estimate = (%f, %f, %f)",
    //         the_tf_estimate.getOrigin().x(),
    //         the_tf_estimate.getOrigin().y(),
    //         the_tf_estimate.getOrigin().z());

    static tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(
        tf::StampedTransform(
            the_tf_estimate,
            ros::Time::now(),
            "/map",
            "/the_estimate"));
}

void estimate_location()
{
    the_tf_estimate = the_tf_odom * the_tf_odom_to_drone;

    for (auto marker : the_markers)
    {
        if (!marker->used)
        {
            // tf::Vector3 position = tf_drone.getOrigin();
            // tf::Quaternion orientation = tf_drone.getRotation();

            // position = (1.0 - the_marker_23.sigma) * position + the_marker_23.sigma * the_marker_23.tf_to_drone.getPosition();
            // orientation = orientation.slerp(the_marker_23.tf_to_drone.getOrientation(), 0.1);

            // // feedback
            // tf_estimate = tf::Transform(estimate_rotation, estimate_position);
            // the_tf_odom = tf_drone * tf_odometry.inverse();

            the_tf_odom = marker->absolute_position * marker->tf_to_drone * the_tf_odom_to_drone.inverse();
            the_tf_estimate = marker->absolute_position * marker->tf_to_drone;
            marker->used = true;
        }
    }

    publish_estimate();
}

void initialize_topics(ros::NodeHandle & handle)
{
    // Public Interface
    odometry_topic = handle.subscribe<geometry_msgs::TransformStamped>("odometry", 10, process_odometry);
    markers_topic = handle.subscribe<geometry_msgs::TransformStamped>("/marker_detector/markers", 10, process_marker);
    estimate_topic = handle.advertise<geometry_msgs::TransformStamped>("estimate", 10);
}

void initialize_transforms(const ros::NodeHandle & handle)
{
    // handle.param<std::string>("base_frame_id", the_base_frame_id, "/ardrone_base_link");
    // handle.param<std::string>("camera_frame_id", the_camera_frame_id, "/ardrone_base_bottomcam");

    // BaselinkToCamera = read_transform(the_camera_frame_id, the_base_frame_id);

    // ArDrone OK
    tf_drone_to_camera = tf::Transform(tf::Quaternion(-0.7071, 0.7071, 0.0, 0.0), tf::Vector3(0.0, 0.0, 0.0));

    the_tf_odom = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0.0, 0.0, 0.0));
    the_tf_odom_to_drone = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0.0, 0.0, 0.0));
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "localization");

    ros::NodeHandle handle("~");
    initialize_topics(handle);
    initialize_transforms(handle);

    ros::Rate loop_rate(60); // Hz
    while (ros::ok())
    {
        estimate_location();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}




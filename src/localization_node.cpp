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


struct LandMark
{
    LandMark(double x, double y)
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
        : landmark_428(0, 0)
        , landmark_341(0, 0)
        , landmark_985(2, 0)
        , odometry_topic(nh.subscribe<geometry_msgs::TransformStamped>("odometry", 10, &LocalizationNode::onOdometry, this))
        , markers_topic(nh.subscribe<geometry_msgs::TransformStamped>("/marker_detector/markers", 10, &LocalizationNode::onMarker, this))
        , estimate_topic(nh.advertise<geometry_msgs::TransformStamped>("odometry", 10))
    {
        known_landmarks = std::vector<LandMark*> {&landmark_428, &landmark_341, &landmark_985};

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

            for (auto marker : known_landmarks)
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
        LandMark* landmark = identifyLandMark(camera_to_marker_msg.child_frame_id);
        if (!landmark) return;

        tf::StampedTransform tf_camera_to_marker;
        tf::transformStampedMsgToTF(camera_to_marker_msg, tf_camera_to_marker);
        tf::Transform tf_marker_to_drone = (tf_drone_to_camera * tf_camera_to_marker).inverse();

        tf::Vector3 position = tf_marker_to_drone.getOrigin();
        tf::Quaternion orientation = tf_marker_to_drone.getRotation();

        if (landmark->frames > 0)
        {
            position = (position + landmark->tf_to_drone.getOrigin() * landmark->frames) / (1. + landmark->frames);
            orientation = landmark->tf_to_drone.getRotation().slerp(orientation, 1.0 / landmark->frames);
            landmark->frames = landmark->frames < 10 ? landmark->frames + 1 : landmark->frames;
        }
        else
        {
            landmark->frames = 1;
        }

        landmark->used = false;

        landmark->tf_to_drone.setOrigin(position);
        landmark->tf_to_drone.setRotation(orientation);
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

    LandMark* identifyLandMark(const std::string& id)
    {
        if ("marker_428" == id)
        {
            return &landmark_428;
        }
        else if ("marker_341" == id)
        {
            return &landmark_341;
        }
        else if ("marker_985" == id)
        {
            return &landmark_985;
        }
        return nullptr;
    }

    LandMark landmark_428;
    LandMark landmark_341;
    LandMark landmark_985;
    std::vector<LandMark*> known_landmarks;

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

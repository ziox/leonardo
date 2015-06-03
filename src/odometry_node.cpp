#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ardrone_autonomy/Navdata.h>
#include <cmath>


// Zero
tf::Vector3 start_zero;
double actual_yaw = 0.0;
double zero_yaw = 0.0;
double x_on_x = 1.0;
double x_on_y = 0.0;
double y_on_y = 1.0;
double y_on_x = 0.0;

// Drone State
tf::Vector3 position;
tf::Quaternion orientation;
ros::Time timestamp;
double tm = -1;

// Input
ros::Subscriber twist_topic;
ros::Subscriber navdata_topic;
ros::Subscriber command_topic;

// Output
ros::Publisher odometry_topic;
ros::Publisher cmd_vel_topic;
ros::Publisher takeoff_topic;
ros::Publisher land_topic;


void process_command(std_msgs::String msg)
{
    std::string command = msg.data;
    if (command == "start zero")
    {
        ROS_ERROR("START ZERO @ (%f, %f, %f)", position.x(), position.y(), position.z());
        start_zero = position;
    }
    else if (command == "end zero")
    {
        auto ds = tf::Vector3(position.x() - start_zero.x(), position.y() - start_zero.y(), 0.);
        if (std::fabs(ds.length()) > 0.1)
        {
            x_on_x = tf::Vector3(1., 0., 0.).dot(ds) / ds.length();
            x_on_y = tf::Vector3(0., 1., 0.).dot(ds) / ds.length();
            y_on_x = -x_on_y;
            y_on_y = x_on_x;

            // auto yaw = ds.normalize().angle(tf::Vector3(1., 0., 0.));
            // yaw = x_on_y > 0.0 ? -zero_yaw : zero_yaw;

            zero_yaw = actual_yaw;

        }

        ROS_ERROR("STOP ZERO @ (%f, %f, %f)", position.x(), position.y(), position.z());
        ROS_ERROR("correction = [%f, %f; %f, %f]", x_on_x, x_on_y, y_on_x, y_on_y);
    }
    else if (command == "zero")
    {
        zero_yaw = actual_yaw;
        position = tf::Vector3(0, 0, 0);
    }
}

void publish_odometry()
{
    tf::Transform tf_transform(orientation, position);
    geometry_msgs::Transform transform;
    tf::transformTFToMsg(tf_transform, transform);

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = timestamp;
    transform_stamped.transform = transform;

    odometry_topic.publish<geometry_msgs::TransformStamped>(transform_stamped);

    // static tf::Transform tf_correction(
    //     tf::createQuaternionFromRPY(0, 0, 3.14/2),
    //     tf::Vector3(0, 0, 1));
    static tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf_transform,
            ros::Time::now(),
            "/map",
            "/the_odometry_reference"));
    // broadcaster.sendTransform(
    //     tf::StampedTransform(
    //         tf_correction * tf_pose,
    //         ros::Time::now(),
    //         "/map",
    //         "/the_corrected_odometry"));
}

void process_navdata(ardrone_autonomy::Navdata navdata)
{
    if (tm < 0)
    {
        tm = navdata.tm;
        return; // DO NOTHING!
    }

    // navdata.tm [us]
    double dt_us = navdata.tm - tm;
    if (dt_us < 0)
    {
        ROS_ERROR("NEGATIVE TIME");
        tm = navdata.tm;
        return;
    }

    double dt_ns = dt_us * 1.e3;

    // double dt_ns = navdata.tm * 1.e3 - timestamp.toNSec();
    // dt_ns = dt_ns < 5e6 ? dt_ns : 5e6; // limit dt to 5ms (200Hz)

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


    // double x = position.x() + (-navdata.vy) * dt_ns / 1.e12;
    // double x = position.x() + dx * x_on_x + dy * y_on_x;
    // double y = position.y() + (navdata.vx) * dt_ns / 1.e12;
    // double y = position.y() + dx * x_on_y + dy * y_on_y;
    // navdata.altd [mm]
    double z = navdata.altd / 1.e3;

    orientation = tf::createQuaternionFromRPY(roll, pitch, yaw - zero_yaw);
    tf::Vector3 ds(dx, dy, 0.);
    ds = ds.rotate(tf::Vector3(0., 0., 1.), yaw - zero_yaw);
    // orientation = tf::createIdentityQuaternion();
    position = tf::Vector3(position.x() + ds.x(), position.y() + ds.y(), z);
    timestamp.fromNSec(navdata.tm * 1e3);
    tm = navdata.tm;

    publish_odometry();
}

void process_twist(geometry_msgs::Twist twist)
{
    cmd_vel_topic.publish(twist);
}

void initialize_topics(ros::NodeHandle & handle)
{
    // Public Interface
    odometry_topic = handle.advertise<geometry_msgs::TransformStamped>("odometry", 10);
    twist_topic = handle.subscribe<geometry_msgs::Twist>("twist", 10, process_twist);
    command_topic = handle.subscribe<std_msgs::String>("command", 10, process_command);

    // Implementation
    cmd_vel_topic = handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    takeoff_topic = handle.advertise<std_msgs::Empty>("/ardrone/takeoff", 10);
    land_topic = handle.advertise<std_msgs::Empty>("/ardrone/land", 10);
    navdata_topic = handle.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata", 10, process_navdata);
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "odometry");

    ros::NodeHandle handle("~");
    initialize_topics(handle);

    ros::spin();

    return 0;
}

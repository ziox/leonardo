#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_datatypes.h>
#include "utils.h"
#include <cmath>

ros::Subscriber estimate_topic;
ros::Subscriber set_point_topic;
ros::Publisher twist_topic;
ros::Subscriber command_topic;
ros::Publisher land_topic;
ros::Publisher takeoff_topic;

tf::Transform the_tf_estimate[2];
double the_time_ns[2];
tf::Transform the_tf_setpoint;

bool active = false;


void process_estimate(geometry_msgs::TransformStamped estimate_msg)
{
    the_tf_estimate[1] = the_tf_estimate[0];
    the_time_ns[1] = the_time_ns[0];
    tf::transformMsgToTF(estimate_msg.transform, the_tf_estimate[0]);
    the_time_ns[0] = ros::Time::now().toNSec();
}

void process_set_point(geometry_msgs::Point msg)
{
    the_tf_setpoint.setOrigin(tf::Vector3(msg.x, msg.y, msg.z));
}

void hover()
{
    ROS_ERROR("HOVER");
    active = false;
    geometry_msgs::Twist HoverTwist;
    HoverTwist.linear.x = 0.0;
    HoverTwist.linear.y = 0.0;
    HoverTwist.linear.z = 0.0;
    HoverTwist.angular.x = 0.0;
    HoverTwist.angular.y = 0.0;
    HoverTwist.angular.z = 0.0;
    twist_topic.publish(HoverTwist);
}

void process_command(std_msgs::String msg)
{
    std::string command = msg.data;
    if (command == "takeoff")
    {
        active = false;
        takeoff_topic.publish(std_msgs::Empty());
    }
    else if (command == "land")
    {
        hover();
        land_topic.publish(std_msgs::Empty());
    }
    else if (command == "hover")
    {
        hover();
    }
    else if (command == "toggle_ctrl")
    {
        if (active)
        {
            hover();
        }
        else
        {
            ROS_ERROR("ACTIVE");
            active = true;
        }
    }
}

void initialize_topics(ros::NodeHandle & handle)
{
    estimate_topic = handle.subscribe<geometry_msgs::TransformStamped>("estimate", 10, process_estimate);
    set_point_topic = handle.subscribe<geometry_msgs::Point>("set_point", 10, process_set_point);
    twist_topic = handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    command_topic = handle.subscribe<std_msgs::String>("command", 10, process_command);
    land_topic = handle.advertise<std_msgs::Empty>("/ardrone/land", 10);
    takeoff_topic = handle.advertise<std_msgs::Empty>("/ardrone/takeoff", 10);
}

void initialize_transforms(const ros::NodeHandle & handle)
{
    the_tf_estimate[0] = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0.0, 0.0, 0.0));
    the_tf_estimate[1] = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0.0, 0.0, 0.0));
    the_tf_setpoint = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0.0, 0.0, 1.0));
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle handle("~");
    double kP = 0.2;
    handle.param<double>("kp", kP, 0.2);
    double kD = 0.2;
    handle.param<double>("kd", kD, 0.2);

    initialize_topics(handle);
    initialize_transforms(handle);

    MAFilter<3> x_speed;
    MAFilter<3> y_speed;

    ros::Rate loop_rate(60); // Hz
    while (ros::ok())
    {
        tf::Transform reference(the_tf_estimate[0].inverse().getRotation(), tf::Vector3(0., 0., 0.));

        tf::Vector3 direction = reference * tf::Vector3(1., 0., 0.);
        double angle = tf::Vector3(1., 0., 0.).angle(direction);
        angle = direction.y() < 0 ? angle : -angle;

        tf::Vector3 error = the_tf_setpoint.getOrigin() - the_tf_estimate[0].getOrigin();
        error = reference * error;

        double dt = (the_time_ns[0] - the_time_ns[1]) / 1.e9;
        tf::Vector3 velocity = (the_tf_estimate[0].getOrigin() - the_tf_estimate[1].getOrigin()) / dt;
        if (fabs(dt) < 0.0001)
        {
            velocity = tf::Vector3(0., 0., 0.);
        }
        velocity = reference * velocity;

        double dx = error.x();
        double dy = error.y();
        double dz = error.z();

        double vx = x_speed.filter(velocity.x());
        double vy = y_speed.filter(velocity.y());

        double ctrl_x = kP * dx - kD * vx;
        double ctrl_y = kP * dy - kD * vy;
        double ctrl_z = dz;

        // ROS_ERROR("Angle:%f", angle);
        // ROS_ERROR("X:%f Y:%f Z:%f", the_tf_estimate[0].getOrigin().x(), the_tf_estimate[0].getOrigin().y(), the_tf_estimate[0].getOrigin().z());
        // ROS_ERROR("DX:%f DY:%f DZ:%f", dx, dy, dz);
        // ROS_ERROR("VX:%f VY:%f", vx, vy);
        // ROS_ERROR("CX:%f CY:%f CZ:%f", ctrl_x, ctrl_y, ctrl_z);

        constexpr double MIN_POWER = 0.01;
        constexpr double MAX_POWER = 0.30;

        if (active)
        {
            ctrl_x = std::max(std::min(ctrl_x, MAX_POWER), -MAX_POWER);
            ctrl_x = fabs(ctrl_x) > MIN_POWER ? ctrl_x : 0.0;
            ctrl_y = std::max(std::min(ctrl_y, MAX_POWER), -MAX_POWER);
            ctrl_y = fabs(ctrl_y) > MIN_POWER ? ctrl_y : 0.0;
            ctrl_z = fabs(ctrl_z) > MIN_POWER ? ctrl_z : 0.0;

            //ROS_ERROR("CX:%f CY:%f CZ:%f", ctrl_x, ctrl_y, ctrl_z);

            geometry_msgs::Twist twist;
            twist.linear.x = ctrl_x;
            twist.linear.y = ctrl_y;
            twist.linear.z = ctrl_z;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;
            twist_topic.publish(twist);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

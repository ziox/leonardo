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

struct ControllerNode
{
    ControllerNode(ros::NodeHandle& nh)
        : active(false)
        , threshold(0.01)
        , limit(0.30)
        , set_point(tf::createIdentityQuaternion(), tf::Vector3(0.0, 0.0, 1.0))
        , estimate_topic(nh.subscribe<geometry_msgs::TransformStamped>("estimate", 10, &ControllerNode::onEstimate, this))
        , set_point_topic(nh.subscribe<geometry_msgs::Point>("set_point", 10, &ControllerNode::onSetPoint, this))
        , twist_topic(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10))
        , command_topic(nh.subscribe<std_msgs::String>("command", 10, &ControllerNode::onCommand, this))
        , land_topic(nh.advertise<std_msgs::Empty>("/ardrone/land", 10))
        , takeoff_topic(nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 10))
    {
        nh.param<double>("kp", kP, 0.2);
        nh.param<double>("kd", kD, 0.2);
    }

    void run()
    {
        ros::Rate loop_rate(60); // Hz
        while (ros::ok())
        {
            auto error = currentRelativeError();
            auto velocity = currentRelativeVelocity();

            double dx = error.x();
            double dy = error.y();
            double dz = error.z();

            double vx = x_speed_filter(velocity.x());
            double vy = y_speed_filter(velocity.y());

            double ctrl_x = - (kP * dx + kD * vx);
            double ctrl_y = - (kP * dy + kD * vy);
            double ctrl_z = - dz;

            if (active)
            {
                ctrl_x = threshold(limit(ctrl_x));
                ctrl_y = threshold(limit(ctrl_y));
                ctrl_z = threshold(ctrl_z);
                sendControlCommand(ctrl_x, ctrl_y, ctrl_z);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    tf::Transform currentDroneReference()
    {
        return tf::Transform(current_belief.pose.inverse().getRotation(), tf::Vector3(0., 0., 0.));
    }

    tf::Vector3 currentRelativeError()
    {
        return currentDroneReference() * (current_belief.pose.getOrigin() - set_point.getOrigin());
    }

    tf::Vector3 currentRelativeVelocity()
    {
        double dt = (current_belief.time - previous_belief.time) / 1.e9;
        tf::Vector3 velocity = (current_belief.pose.getOrigin() - previous_belief.pose.getOrigin()) / dt;
        if (fabs(dt) < 0.0001)
        {
            return tf::Vector3(0., 0., 0.);
        }
        return currentDroneReference() * velocity;
    }

    void sendControlCommand(double ctrl_x, double ctrl_y, double ctrl_z)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = ctrl_x;
        twist.linear.y = ctrl_y;
        twist.linear.z = ctrl_z;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
        twist_topic.publish(twist);
    }

    void onEstimate(geometry_msgs::TransformStamped estimate_msg)
    {
        previous_belief.pose = current_belief.pose;
        previous_belief.time = current_belief.time;
        tf::transformMsgToTF(estimate_msg.transform, current_belief.pose);
        current_belief.time = ros::Time::now().toNSec();
    }

    void onSetPoint(geometry_msgs::Point msg)
    {
        set_point.setOrigin(tf::Vector3(msg.x, msg.y, msg.z));
    }

    void onCommand(std_msgs::String msg)
    {
        std::string command = msg.data;
        if (command == "takeoff")
        {
            takeoff();
        }
        else if (command == "land")
        {
            land();
        }
        else if (command == "hover")
        {
            hover();
        }
        else if (command == "toggle_ctrl")
        {
            toggleCtrl();
        }
    }

    void takeoff()
    {
        active = false;
        takeoff_topic.publish(std_msgs::Empty());
    }

    void hover()
    {
        active = false;
        sendControlCommand(0, 0, 0);
    }

    void land()
    {
        hover();
        land_topic.publish(std_msgs::Empty());
    }

    void toggleCtrl()
    {
        if (active)
        {
            hover();
        }
        else
        {
            active = true;
        }
    }

    bool active;
    double kP;
    double kD;

    MAFilter<3> x_speed_filter;
    MAFilter<3> y_speed_filter;
    Threshold threshold;
    Limit limit;

    struct Belief
    {
        Belief(tf::Transform pose = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0)) , double time = 0)
            : pose(pose)
            , time(time)
        {}

        tf::Transform pose;
        double time;
    };

    Belief current_belief;
    Belief previous_belief;
    tf::Transform set_point;

    ros::Subscriber estimate_topic;
    ros::Subscriber set_point_topic;
    ros::Publisher twist_topic;
    ros::Subscriber command_topic;
    ros::Publisher land_topic;
    ros::Publisher takeoff_topic;
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle handle("~");
    ControllerNode(handle).run();
}

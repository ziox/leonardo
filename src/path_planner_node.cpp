#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "utils.h"

ros::Subscriber command_topic;
ros::Publisher set_point_topic;

tf::Vector3 the_tf_setpoints[10];

tf::Vector3 start;
tf::Vector3 actual;
tf::Vector3 end;

const int TAU_MAX = 100;
int the_tau = 100;


void process_command(std_msgs::String msg)
{
    std::string command = msg.data;
    if (command.substr(0, 5) == "point")
    {
        int index = atoi(&command[6]);
        end = the_tf_setpoints[index];
        start = actual;
        the_tau = 0;
    }
}

void publish_set_point()
{
    static tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::createIdentityQuaternion(), actual),
            ros::Time::now(),
            "/map",
            "/the_drone"));
}

void initialize_transforms(const ros::NodeHandle & handle)
{
    the_tf_setpoints[0] = tf::Vector3(0.0, 0.0, 1.0);

    the_tf_setpoints[1] = tf::Vector3(0.0, 0.0, 1.0);
    the_tf_setpoints[2] = tf::Vector3(0.0, 3.0, 1.0);
    the_tf_setpoints[3] = tf::Vector3(2.0, 3.0, 1.0);
    the_tf_setpoints[4] = tf::Vector3(2.0, 0.0, 1.0);

    // Triangle
    // the_tf_setpoints[1] = tf::Vector3(0.0, 0.0, 1.0);
    // the_tf_setpoints[2] = tf::Vector3(0.7, 2.7, 1.0);
    // the_tf_setpoints[3] = tf::Vector3(2.0, 0.0, 1.0);


    the_tf_setpoints[5] = tf::Vector3(0.0, 0.0, 2.5);
    the_tf_setpoints[6] = tf::Vector3(0.0, 3.0, 2.5);
    the_tf_setpoints[7] = tf::Vector3(2.0, 3.0, 2.5);
    the_tf_setpoints[8] = tf::Vector3(2.0, 0.0, 2.5);

    the_tf_setpoints[9] = tf::Vector3(1.0, 0.0, 2.5);

    start = the_tf_setpoints[0];
    actual = start;
    end = start;
}

void initialize_topics(ros::NodeHandle & handle)
{
    // Public Interface
    command_topic = handle.subscribe<std_msgs::String>("command", 10, process_command);
    set_point_topic = handle.advertise<geometry_msgs::Point>("set_point", 10);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "path_planner");

    ros::NodeHandle handle("~");
    initialize_topics(handle);
    initialize_transforms(handle);

    const MotionProfile& profile = SinProfile{};

    ros::Rate loop_rate(10); // Hz
    while (ros::ok())
    {
        double percent = 1.0;
        double tau  = 3.14 * the_tau / TAU_MAX;
        if (the_tau < TAU_MAX)
        {
            percent = profile.position(tau);
        }

        actual = start + (end - start) * percent;
        the_tau = the_tau < TAU_MAX ? the_tau + 1 : TAU_MAX;

        geometry_msgs::Point cmd;
        cmd.x = actual.x();
        cmd.y = actual.y();
        cmd.z = actual.z();
        set_point_topic.publish(cmd);

        // ROS_ERROR("set point tau:%d x:%f, y:%f, z:%f", the_tau, cmd.x, cmd.y, cmd.z);
        publish_set_point();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

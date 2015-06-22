#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "utils.h"


struct PathPlannerNode
{
    PathPlannerNode(ros::NodeHandle& nh)
        : tick(MaxTick)
        , set_point_topic(nh.advertise<geometry_msgs::Point>("set_point", 10))
        , command_topic(nh.subscribe<std_msgs::String>("command", 10, &PathPlannerNode::onCommand, this))
    {
        the_tf_setpoints[0] = tf::Vector3(0.0, 0.0, 1.0);
        the_tf_setpoints[1] = tf::Vector3(0.0, 0.0, 1.0);
        the_tf_setpoints[2] = tf::Vector3(0.0, 3.0, 1.0);
        the_tf_setpoints[3] = tf::Vector3(2.0, 3.0, 1.0);
        the_tf_setpoints[4] = tf::Vector3(2.0, 0.0, 1.0);
        the_tf_setpoints[5] = tf::Vector3(0.0, 0.0, 2.5);
        the_tf_setpoints[6] = tf::Vector3(0.0, 3.0, 2.5);
        the_tf_setpoints[7] = tf::Vector3(2.0, 3.0, 2.5);
        the_tf_setpoints[8] = tf::Vector3(2.0, 0.0, 2.5);
        the_tf_setpoints[9] = tf::Vector3(1.0, 0.0, 2.5);

        start = the_tf_setpoints[0];
        actual = start;
        end = start;
    }

    void run()
    {
        ros::Rate loop_rate(10); // Hz
        while (ros::ok())
        {
            if (tick <= MaxTick)
            {
                actual = start + (end - start) * profile.position(1.0 * tick / MaxTick);
                publishSetPoint();
                ++tick;
            }

            broadcastSetPoint();

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void onCommand(std_msgs::String msg)
    {
        std::string command = msg.data;
        if (command.substr(0, 5) == "point")
        {
            int index = atoi(&command[6]);
            end = the_tf_setpoints[index];
            start = actual;
            tick = 0;
        }
    }

    void publishSetPoint()
    {
        geometry_msgs::Point cmd;
        cmd.x = actual.x();
        cmd.y = actual.y();
        cmd.z = actual.z();
        set_point_topic.publish(cmd);
    }

    void broadcastSetPoint()
    {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::createIdentityQuaternion(), actual),
                ros::Time::now(),
                "/map",
                "/the_set_point"));
    }

    int tick;
    static constexpr int MaxTick = 100;
    SinProfile profile;

    tf::Vector3 start;
    tf::Vector3 actual;
    tf::Vector3 end;
    tf::Vector3 the_tf_setpoints[10];

    ros::Subscriber command_topic;
    ros::Publisher set_point_topic;
    tf::TransformBroadcaster broadcaster;
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle handle("~");
    PathPlannerNode(handle).run();
}

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <keyboard/Key.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <csignal>


ros::Publisher cmd_vel_topic;
ros::Publisher command_topic;

bool running = true;

void terminate(int sig)
{
    running = false;
}

struct
{
    bool up;
    bool down;
    bool forward;
    bool backward;
    bool left;
    bool right;
    bool cw;
    bool acw;
} the_key_status;

bool active = true;



void process_keydown(keyboard::Key key)
{
    // ROS_ERROR("keydown: %d", key.code);

    std_msgs::String cmd;
    switch(key.code)
    {
    case 101: // e
        cmd.data = "toggle_ctrl";
        command_topic.publish(cmd);
        active = !active;
        break;
    case 108: // l
        cmd.data = "land";
        command_topic.publish(cmd);
        break;
    case 116: // t
        cmd.data = "takeoff";
        command_topic.publish(cmd);
        break;
    case 113: // q
        cmd.data = "land";
        command_topic.publish(cmd);
        running = false;
        break;
    case 48:
    case 49:
    case 50:
    case 51:
    case 52:
    case 53:
    case 54:
    case 55:
    case 56:
    case 57:
        cmd.data = std::string("point ") + std::to_string(key.code - 48);
        command_topic.publish(cmd);
        break;
    case 122:
        cmd.data = "zero";
        command_topic.publish(cmd);
        break;
    case 273:
        the_key_status.forward = true;
        break;
    case 274:
        the_key_status.backward = true;
        break;
    case 276:
        the_key_status.left = true;
        break;
    case 275:
        the_key_status.right = true;
        break;
    case 119: // w
        the_key_status.up = true;
        break;
    case 115: // s
        the_key_status.down = true;
        break;
    case 97: // a
        the_key_status.acw = true;
        break;
    case 100: // d
        the_key_status.cw = true;
        break;
    }
}

void process_keyup(keyboard::Key key)
{
    // ROS_ERROR("keyup: %d", key.code);
    switch(key.code)
    {
    case 273:
        the_key_status.forward = false;
        break;
    case 274:
        the_key_status.backward = false;
        break;
    case 276:
        the_key_status.left = false;
        break;
    case 275:
        the_key_status.right = false;
        break;
    case 119: // w
        the_key_status.up = false;
        break;
    case 115: // s
        the_key_status.down = false;
        break;
    case 97: // a
        the_key_status.acw = false;
        break;
    case 100: // d
        the_key_status.cw = false;
        break;
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tele_poppe");
    std::signal(SIGINT, terminate);
    ros::NodeHandle handle("~");

    cmd_vel_topic = handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    command_topic = handle.advertise<std_msgs::String>("command", 10);

    ros::Subscriber keydown_topic = handle.subscribe<keyboard::Key>("/keyboard/keydown", 10, process_keydown);
    ros::Subscriber keyup_topic = handle.subscribe<keyboard::Key>("/keyboard/keyup", 10, process_keyup);

    const double POWER = 0.20;

    ros::Rate loop_rate(120); // Hz
    while(running)
    {
        double x = 0;
        double y = 0;
        double z = 0;
        double turn = 0;

        if (the_key_status.forward) x += POWER;
        if (the_key_status.backward) x -= POWER;
        if (the_key_status.left) y += POWER;
        if (the_key_status.right) y -= POWER;
        if (the_key_status.up) z += 0.40;
        if (the_key_status.down) z -= 0.40;
        if (the_key_status.cw) turn -= POWER;
        if (the_key_status.acw) turn += POWER;

        geometry_msgs::Twist twist;
        twist.linear.x = x;
        twist.linear.y = y;
        twist.linear.z = z;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = turn;

        if (active)
        {
            cmd_vel_topic.publish(twist);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <keyboard/Key.h>
#include <std_msgs/String.h>


class TeleOp
{
public:
    TeleOp(ros::NodeHandle& nh)
        : keydown_topic(nh.subscribe<keyboard::Key>("/keyboard/keydown", 10, &TeleOp::onKeyDown, this))
        , keyup_topic(nh.subscribe<keyboard::Key>("/keyboard/keyup", 10, &TeleOp::onKeyUp, this))
        , cmd_vel_topic(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10))
        , command_topic(nh.advertise<std_msgs::String>("command", 10))
        , active(true)
    {
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
    }

    ~TeleOp()
    {
        if (active)
        {
            toggleControl();
        }
        land();
    }

    void process()
    {
        if (active)
        {
            cmd_vel_topic.publish(twist);
        }
    }

private:
    void onKeyDown(keyboard::Key key)
    {
        switch (key.code)
        {
        case 101: // e
            toggleControl();
            break;
        case 108: // l
            land();
            break;
        case 116: // t
            takeoff();
            break;
        case 113: // q
            land();
            ros::shutdown();
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
            setPoint(key.code - 48);
            break;
        case 122:
            zero();
            break;
        case 273:
            twist.linear.x += Power;
            break;
        case 274:
            twist.linear.x -= Power;
            break;
        case 276:
            twist.linear.y += Power;
            break;
        case 275:
            twist.linear.y -= Power;
            break;
        case 119: // w
            twist.linear.z += 0.40;
            break;
        case 115: // s
            twist.linear.z -= 0.40;
            break;
        case 97: // a
            twist.angular.z += Power;
            break;
        case 100: // d
            twist.angular.z -= Power;
            break;
        }
    }

    void onKeyUp(keyboard::Key key)
    {
        switch (key.code)
        {
        case 273:
            twist.linear.x -= Power;
            break;
        case 274:
            twist.linear.x += Power;
            break;
        case 276:
            twist.linear.y -= Power;
            break;
        case 275:
            twist.linear.y += Power;
            break;
        case 119: // w
            twist.linear.z -= 0.40;
            break;
        case 115: // s
            twist.linear.z += 0.40;
            break;
        case 97: // a
            twist.angular.z -= Power;
            break;
        case 100: // d
            twist.angular.z += Power;
            break;
        }
    }

    void toggleControl()
    {
        std_msgs::String cmd;
        cmd.data = "toggle_ctrl";
        command_topic.publish(cmd);
        active = !active;
    }

    void takeoff()
    {
        std_msgs::String cmd;
        cmd.data = "takeoff";
        command_topic.publish(cmd);
    }

    void land()
    {
        std_msgs::String cmd;
        cmd.data = "land";
        command_topic.publish(cmd);
    }

    void zero()
    {
        std_msgs::String cmd;
        cmd.data = "zero";
        command_topic.publish(cmd);
    }

    void setPoint(int id)
    {
        std_msgs::String cmd;
        cmd.data = std::string("point ") + std::to_string(id);
        command_topic.publish(cmd);
    }

    ros::Subscriber keydown_topic;
    ros::Subscriber keyup_topic;
    ros::Publisher cmd_vel_topic;
    ros::Publisher command_topic;

    bool active;
    geometry_msgs::Twist twist;

    constexpr static auto Power = 0.20;
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "teleop");
    ros::NodeHandle handle("~");
    TeleOp node(handle);

    ros::Rate loop_rate(120); // Hz
    while (ros::ok())
    {
        node.process();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Anja Sheppard
// April 8, 2019
// Uses the ros-keyboard package to publish throttle and steering commands to the /ecu topic for the sim package

#include "ros/ros.h"
#include "barc/ECU.h"
#include "keyboard/Key.h"

barc::ECU ecu_msg;

void callbackKeyDown (const keyboard::Key msg)
{
    if (msg.code == 119) // 'w'
    {
        ecu_msg.motor = 1;
    }
    else if (msg.code == 100) // 'd'
    {
        ecu_msg.servo = ecu_msg.servo + 1;
    }
    else if (msg.code == 115) // 's'
    {
        ecu_msg.motor = -1;
    }
    else if (msg.code == 97) // 'a'
    {
        ecu_msg.servo = ecu_msg.servo - 1;
    }
}

void callbackKeyUp (const keyboard::Key msg)
{
    if (msg.code == 119) // 'w'
    {
        ecu_msg.motor = 0;
    }
    else if (msg.code == 100) // 'd'
    {
        // just stop incrementing servo degree
    }
    else if (msg.code == 115) // 's'
    {
        ecu_msg.motor = 0;
    }
    else if (msg.code == 97) // 'a'
    {
        // just stop decrementing servo degree
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_manual");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<barc::ECU>("ecu", 1000);
    ros::Subscriber sub_down = n.subscribe("keyboard/keydown", 1000, callbackKeyDown);
    ros::Subscriber sub_up = n.subscribe("keyboard/keyup", 1000, callbackKeyUp);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        pub.publish(ecu_msg);
       
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

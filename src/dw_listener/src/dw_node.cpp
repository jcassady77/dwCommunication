#include "serial/serial.h"
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dw");
    ros::NodeHandle n;

    ros::Publisher dw_pub = n.advertise<std_msgs::String>("dw_data", 1000);

    ros::Rate loop_rate(10);

    try {
        serial::Serial monitor("/dev/ttyACM0", 115200);

        std_msgs::String msg;

        if(monitor.isOpen()) {
        } else {
            monitor.open();
        }
        size_t bytes_wrote = monitor.write("node");

        while (ros::ok()){

            msg.data = "[[[" + monitor.read(200) + "]]]";
            try {
                msg.data = "{" + msg.data.substr(17, msg.data.size()-4);
            } catch(std::exception& err) {
                ROS_INFO("Incompatable Message Length:  %s", err.what());
            }
            ROS_INFO("%s", msg.data.c_str());

            //std_msgs::map<string, auto> readData = msg.data
            dw_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    } catch(std::exception& err) {
        ROS_INFO("ERROR will robinson! %s", err.what());
    }

    return 0;

}

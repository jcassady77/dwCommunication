#include "serial/serial.h"
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <dw_listener/nodeData.h>


int main(int argc, char **argv)
{
    //Iniitalize ros structs
    ros::init(argc, argv, "dw");
    ros::NodeHandle n;
    ros::Publisher dw_pub = n.advertise<dw_listener::nodeData>("dw_data", 1000);
    ros::Rate loop_rate(10);

    try {
        //Set up monitor for serial port
        serial::Serial monitor("/dev/ttyACM0", 115200);
        std_msgs::String msg;
        dw_listener::nodeData msg1;
        //if(monitor.isOpen()) {
        //} else {
        //    monitor.open();
        //}
        size_t bytes_wrote = monitor.write("node");

        while (ros::ok()){
            //Reading serial port
            msg.data = monitor.read(200);
            //Simplifying the input a little
            try {
                msg.data = msg.data.substr(14, msg.data.size()-4);
            } catch(std::exception& err) {
                ROS_INFO("Incompatable Message Length:  %s", err.what());
            }
            //Displays raw string output post simplification
            ROS_INFO("%s", msg.data.c_str());
            
            //Assign specific values to the nice message struct
            try {
                msg1.tagAddress = msg.data.substr(msg.data.find("a16")+6, (msg.data.find("R")-msg.data.find("a16")-9));
                msg1.rangeNum = stoi(msg.data.substr(msg.data.find("R")+3, (msg.data.find("T")-msg.data.find("R")-5)));
                ROS_INFO("%s", msg.data.substr(msg.data.find("R")+3, (msg.data.find("T")-msg.data.find("R")-5)).c_str());
                

            } catch(std::exception& err) {
                ROS_INFO("Error while building nodeData:  %s", err.what());
            }

            dw_pub.publish(msg1);
            ros::spinOnce();
            loop_rate.sleep();
        }
    } catch(std::exception& err) {
        ROS_INFO("ERROR will robinson! %s", err.what());
    }

    return 0;

}

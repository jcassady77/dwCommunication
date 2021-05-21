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
            //ROS_INFO("%s", msg.data.c_str());
            
            //Assign specific values to the nice message struct
            try {
                msg1.tagAddress = msg.data.substr(msg.data.find("a16")+6, (msg.data.find("R")-msg.data.find("a16")-9));
                msg1.rangeNum = stoi(msg.data.substr(msg.data.find("R")+3, (msg.data.find("T")-msg.data.find("R")-5)));
                msg1.timeOfReception = stoi(msg.data.substr(msg.data.find("T")+3, (msg.data.find("D")-msg.data.find("T")-5)));
                msg1.distance = stoi(msg.data.substr(msg.data.find("D")+3, (msg.data.find("P")-msg.data.find("D")-5)));
                msg1.degrees = stoi(msg.data.substr(msg.data.find("P")+3, (msg.data.find("Xcm")-msg.data.find("P")-5)));
                msg1.Xcoord = stoi(msg.data.substr(msg.data.find("Xcm")+5, (msg.data.find("Ycm")-msg.data.find("Xcm")-7)));
                msg1.Ycoord = stoi(msg.data.substr(msg.data.find("Ycm")+5, (msg.data.find("O")-msg.data.find("Ycm")-7)));
                msg1.clockOffset = stoi(msg.data.substr(msg.data.find("O")+3, (msg.data.find("V")-msg.data.find("O")-5)));
                msg1.serviceData = stoi(msg.data.substr(msg.data.find("V")+3, (msg.data.find("X\"")-msg.data.find("V")-5)));
                msg1.Xaccel = stoi(msg.data.substr(msg.data.find("X\"")+3, (msg.data.find("Y\"")-msg.data.find("X\"")-5)));
                msg1.Yaccel = stoi(msg.data.substr(msg.data.find("Y\"")+3, (msg.data.find("Z\"")-msg.data.find("Y\"")-5)));
                msg1.Zaccel = stoi(msg.data.substr(msg.data.find("Z\"")+3, (msg.data.find("}}")-msg.data.find("Z\"")-3)));

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

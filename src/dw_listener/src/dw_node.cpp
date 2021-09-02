#include "dw_node.h"

#define ARMA_USE_LAPACK
//#include "kalman/kalman_filter.h"
#include <vector>
#include <random>
#include "serial/serial.h"
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "dw_listener/nodeData.h"
#include <cmath>
#include "kalman_filter.h"

#include <visualization_msgs/Marker.h>

#include <armadillo>
#include <stdexcept>
using namespace arma;
using namespace std;
using namespace kf;

dw_listener::nodeData dw_data::buildRosMsg() {
    dw_listener::nodeData msg;
    msg.tagAddress = tagAddress;
    msg.rangeNum = (float) rangeNum;
    msg.timeOfReception = (float) timeOfReception;
    msg.distance = (float) distance;
    msg.degrees = (float) degrees;
    msg.Xcoord = (float) Xcoord;
    msg.Ycoord = (float) Ycoord;
    msg.clockOffset = (float) clockOffset;
    msg.serviceData = (float) serviceData;
    return msg;
}

int countRightBrackets(string s) {
    int count = 0;
    for (int i = 0; i < s.size(); i++)
        if (s[i] == '}') count++;

    return count;
}

int main(int argc, char **argv)
{
    //Iniitalize ros structs
    ros::init(argc, argv, "dw_node");
    ros::NodeHandle n;
    ros::Publisher dw_pub = n.advertise<dw_listener::nodeData>("dw_data", 1000);
    double dt = 0.1;
    ros::Rate loop_rate(dt*100);


    try {
        //Set up monitor for serial port
        serial::Serial monitor("/dev/ttyACM1", 115200);
        std_msgs::String msg;
        
        //if(monitor.isOpen()) {
        //} else {
        //    monitor.open();
        //}
        size_t bytes_wrote = monitor.write("node");

        while (ros::ok()){
            //Reading serial port
            msg.data = monitor.read(10000);
            
            //ROS_INFO("%s", msg.data.c_str());

            //Splitting up messages from different nodes
            int nodeCount = 0;
            try {
                nodeCount = countRightBrackets(msg.data.c_str())/2;
            } catch(std::exception& err) {
                ROS_INFO("Message Parsed Incorrectly. Found %d Right Brackets", countRightBrackets(msg.data.c_str()));
            }   
            std::vector<std_msgs::String> splitMsg;
            int start = 0;
            for (int i = 0; i < nodeCount; i ++) {
                std_msgs::String tempdata;
                tempdata.data = msg.data.substr(start, msg.data.find("}}", start+2)+2);
                splitMsg.push_back(tempdata);
                start = msg.data.find("}}", start+2)+2;
            }
            
            //Generate and send off messages
            try {
                //for each node data
                for (int i = 0; i < splitMsg.size(); i++) {
                    dw_data message;
                    //ROS_INFO("--------------");
                    //Assign specific values to the nice message struct
                    message.tagAddress = splitMsg[i].data.substr(splitMsg[i].data.find("\"a16\"")+7,(splitMsg[i].data.find("\"R\"")-splitMsg[i].data.find("\"a16\""))-9);
                    message.rangeNum = stod(splitMsg[i].data.substr(splitMsg[i].data.find("\"R\"")+4, (splitMsg[i].data.find("\"T\"")-splitMsg[i].data.find("\"R\"")-5)));
                    message.timeOfReception = stod(splitMsg[i].data.substr(splitMsg[i].data.find("\"T\"")+4, (splitMsg[i].data.find("\"D\"")-splitMsg[i].data.find("\"T\"")-5)));
                    message.distance = stod(splitMsg[i].data.substr(splitMsg[i].data.find("\"D\"")+4, (splitMsg[i].data.find("\"P\"")-splitMsg[i].data.find("\"D\"")-5)));
                    message.degrees = stod(splitMsg[i].data.substr(splitMsg[i].data.find("\"P\"")+4, (splitMsg[i].data.find("\"Xcm\"")-splitMsg[i].data.find("\"P\"")-5)));
                    message.Xcoord = stod(splitMsg[i].data.substr(splitMsg[i].data.find("\"Xcm\"")+6, (splitMsg[i].data.find("\"Ycm\"")-splitMsg[i].data.find("\"Xcm\"")-7)));
                    message.Ycoord = stod(splitMsg[i].data.substr(splitMsg[i].data.find("\"Ycm\"")+6, (splitMsg[i].data.find("\"O\"")-splitMsg[i].data.find("\"Ycm\"")-7)));
                    message.clockOffset = stod(splitMsg[i].data.substr(splitMsg[i].data.find("\"O\"")+4, (splitMsg[i].data.find("\"V\"")-splitMsg[i].data.find("\"O\"")-5)));
                    message.serviceData = stod(splitMsg[i].data.substr(splitMsg[i].data.find("\"V\"")+4, (splitMsg[i].data.find("\"X\"")-splitMsg[i].data.find("\"V\"")-5)));
                
                    dw_pub.publish(message.buildRosMsg());
                }
                
                
            } catch(std::exception& err) {
                ROS_INFO("Error while building dw_data:  %s", err.what());
            }

            
            //ROS loop
            ros::spinOnce();
            loop_rate.sleep();
        }
    } catch(std::exception& err) {
        ROS_INFO("ERROR will robinson! %s", err.what());
    }

    return 0;

}

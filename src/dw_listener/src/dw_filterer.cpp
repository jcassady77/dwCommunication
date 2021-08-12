#include "dw_filterer.h"
//#include "dw_node.h"
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
#include "dw_listener/dwFiltered.h"
#include <cmath>

#include "kalman_filter.h"
#include "extended_kalman_filter.h"

#include <visualization_msgs/Marker.h>

#include <armadillo>
#include <stdexcept>
using namespace arma;
using namespace std;
using namespace kf;

dw_data nodeInfo;


dw_listener::dwFiltered dw_filtered::buildRosMsgFilter() {
    dw_listener::dwFiltered msg;
    msg.XcoordGateFiltered = XcoordGateFiltered;
    msg.YcoordGateFiltered = YcoordGateFiltered;
    msg.XcoordKalmanFiltered = XcoordKalmanFiltered;
    msg.YcoordKalmanFiltered = YcoordKalmanFiltered; 
    return msg;
}
void dw_filtered::updateHistory(dw_data msg) {
    int i;
    for (i = FILTER_SIZE-1; i > 0; i--)
    {
        this->xHistory[i] = this->xHistory[i-1];
        this->yHistory[i] = this->yHistory[i-1];
    }
    this->xHistory[0] = msg.Xcoord;
    this->yHistory[0] = msg.Ycoord;
}

void dw_filtered::gateFilter(dw_data msg) {
    int i;
    //Find Means
    int meanX = 0;
    int meanY = 0;
    for (i = FILTER_SIZE-1; i > 0; i--)
    {
        meanX += xHistory[i];
        meanY += yHistory[i];
    }
    meanX /= FILTER_SIZE;
    meanY /= FILTER_SIZE;

    //find stdev
    int stdevX = 0;
    int stdevY = 0;
    for (i = FILTER_SIZE-1; i > 0; i--)
    {
        stdevX += abs(meanX - xHistory[i]);
        stdevY += abs(meanY - yHistory[i]);
    }
    stdevX /= FILTER_SIZE;
    stdevY /= FILTER_SIZE;

    stdevX = 10;
    stdevY = 10;
    //Implement Gate
    if (abs(msg.Xcoord - xHistory[0]) > (stdevX * GATE_VALUE))
    {
        XcoordGateFiltered = xHistory[0];
        //this->Xcoord = this->xHistory[0];
    }
    if (abs(msg.Ycoord - yHistory[0]) > (stdevY * GATE_VALUE))
    {
        YcoordGateFiltered = yHistory[0];
        //this->Ycoord = this->yHistory[0];
    }
    
}


void dw_filtered::gateFilter2(dw_data msg) {
    int i;
    //Find Means
    int meanX = msg.Xcoord;
    int meanY = msg.Ycoord;
    int xCount = 1;
    int yCount = 1;
    for (i = FILTER_SIZE-1; i > 0; i--)
    {
        if (abs(xHistory[i] - xHistory[i+1])> GATE_VALUE)
        {
            meanX += xHistory[i];
            xCount += 1;
        }
        if (abs(yHistory[i] - yHistory[i+1])> GATE_VALUE)
        {
            meanY += yHistory[i];
            yCount += 1;
        }
        
    }
    
    meanX /= xCount;// + 1;
    meanY /= yCount;// + 1;
    
    XcoordGateFiltered = meanX;
    YcoordGateFiltered = meanY;
}
/*
void IMUcallback(const SomePackage::SomeMessageType::ConstPtr& msg)
{
    IMUXaccel.data = msg.linear_acceleration[x]
    IMUYaccel.data = msg.linear_acceleration[y]
    IMUZaccel.data = msg.linear_acceleration[z]
}
*/

void dwCallback(const dw_listener::nodeData::ConstPtr& msg)
{
    nodeInfo.Xcoord = msg->Xcoord;
    nodeInfo.Ycoord = msg->Ycoord;
}
int main(int argc, char **argv)
{
    //Iniitalize ros structs
    ros::init(argc, argv, "dw_filter");
    ros::NodeHandle n;
    ros::Publisher dw_pub = n.advertise<dw_listener::dwFiltered>("dw_filterer", 1000);
    ros::Subscriber dw_sub = n.subscribe("/dw_data", 1000, dwCallback);
    double dt = 0.1;
    ros::Rate loop_rate(dt*100);

    //ros::Subscriber IMUsub = n.subscribe("/imu/data", 100, IMUcallback);

    //Kalman Filter Variables
    double measurement_mu = 0.0;      // Mean
    double measurement_sigma = 0.5;   // Standard deviation

    double process_mu = 0.0;
    double process_sigma = 0.05;

    default_random_engine generator;
    normal_distribution<double> measurement_noise(measurement_mu, measurement_sigma);
    normal_distribution<double> process_noise(process_mu, process_sigma);
    
    double trueXaccel = 0;
    double trueYaccel = 0;
    double trueXvel = 0;
    double trueYvel = 0;
    double trueXDpos = 0;
    double trueYDpos = 0;

    // Preparing KF
    mat A = {   {1.0, 0.0, 0.0, 0.0},
                {dt, 1.0, 0.0, 0.0},
                {0.0, 0.0, 1.0, 0.0},
                {0.0, 0.0, dt, 1.0} 
                };

    mat B = {   {dt, 0.0},
                {dt * dt / 2.0, 0.0},
                {0.0, dt},
                {0.0, dt * dt / 2.0}
                };

    mat C = {   {0.0, 1.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 1.0}
                };
    KalmanFilter kf(A, B, C);

    // The process and measurement covariances are sort of tunning parameters
    mat Q = {   {1.0, 0.0, 0.0, 0.0},
                {0.0, 1.0, 0.0, 0.0},
                {0.0, 0.0, 1.0, 0.0},
                {0.0, 0.0, 0.0, 1.0} 
                };
    mat R = {    {1.0, 0.0},
                {0.0, 1.0}
                };  

    kf.setProcessCovariance(Q);
    kf.setOutputCovariance(R);

    try {

        dw_filtered filteredData; 
        while (ros::ok()){
            filteredData.XcoordGateFiltered = 0;
            filteredData.YcoordGateFiltered = 0;
            filteredData.XcoordKalmanFiltered = 0;
            filteredData.XcoordKalmanFiltered = 0;
                //message.tagAddress.data = msg.data.substr(msg.data.find("a16")+6, (msg.data.find("R")-msg.data.find("a16")-9));
                //message.rangeNum.data = stoi(msg.data.substr(msg.data.find("R")+3, (msg.data.find("T")-msg.data.find("R")-5)));
                //message.timeOfReception.data = stoi(msg.data.substr(msg.data.find("T")+3, (msg.data.find("D")-msg.data.find("T")-5)));
                //message.distance.data = stoi(msg.data.substr(msg.data.find("D")+3, (msg.data.find("P")-msg.data.find("D")-5)));
                //message.degrees.data = stoi(msg.data.substr(msg.data.find("P")+3, (msg.data.find("Xcm")-msg.data.find("P")-5)));
                //message.Xcoord.data = stoi(msg.data.substr(msg.data.find("Xcm")+5, (msg.data.find("Ycm")-msg.data.find("Xcm")-7)));
                //message.Ycoord.data = stoi(msg.data.substr(msg.data.find("Ycm")+5, (msg.data.find("O")-msg.data.find("Ycm")-7)));
                //message.clockOffset.data = stoi(msg.data.substr(msg.data.find("O")+3, (msg.data.find("V")-msg.data.find("O")-5)));
                //message.serviceData.data = stoi(msg.data.substr(msg.data.find("V")+3, (msg.data.find("X\"")-msg.data.find("V")-5)));
                //message.Xaccel.data = stoi(msg.data.substr(msg.data.find("X\"")+3, (msg.data.find("Y\"")-msg.data.find("X\"")-5)));
                //message.Yaccel.data = stoi(msg.data.substr(msg.data.find("Y\"")+3, (msg.data.find("Z\"")-msg.data.find("Y\"")-5)));
                //message.Zaccel.data = stoi(msg.data.substr(msg.data.find("Z\"")+3, (msg.data.find("}}")-msg.data.find("Z\"")-3)));
                
                //message.Xaccel.data = IMUXaccel.data;
                //message.Yaccel.data = IMUYaccel.data;
                //message.Zaccel.data = IMUZaccel.data;
                
            //Filter with gateing 
           
            filteredData.gateFilter2(nodeInfo);
 
            //Kalman Filter

            kf.updateState({
                    static_cast<double>(0), //X accel
                    static_cast<double>(0)}, //Y accel
                    {
                    static_cast<double>(filteredData.XcoordGateFiltered), 
                    static_cast<double>(filteredData.YcoordGateFiltered)});
            
            vec estimate = kf.getEstimate();

            //SEND TO ROS MSG
            filteredData.XcoordKalmanFiltered = estimate(0);
            filteredData.YcoordKalmanFiltered = estimate(1);
            
            dw_pub.publish(filteredData.buildRosMsgFilter());
            filteredData.updateHistory(nodeInfo);
            
            ros::spinOnce();
            loop_rate.sleep();
        }
    } catch(std::exception& err) {
        ROS_INFO("ERROR will robinson! %s", err.what());
    }

    return 0;

}
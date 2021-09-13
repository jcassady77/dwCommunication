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
#include "sensor_msgs/Imu.h"
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

double IMUXaccel;
double IMUYaccel;
double IMUZaccel;

std::string nodeID;

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

void IMUcallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    IMUXaccel = msg->linear_acceleration.x;
    IMUYaccel = msg->linear_acceleration.y;
    IMUZaccel = msg->linear_acceleration.z;
}

void dwCallback(const dw_listener::nodeData::ConstPtr& msg)
{
    if (nodeID.compare(msg->tagAddress) == 0) {
        nodeInfo.tagAddress = msg->tagAddress;
        nodeInfo.Xcoord = msg->Xcoord;
        nodeInfo.Ycoord = msg->Ycoord;
    }
}
int main(int argc, char **argv)
{

    nodeID = (std::string) argv[1];

    //Iniitalize ros structs
    ros::init(argc, argv, "dw_filter");
    ros::NodeHandle n;

    ros::Publisher dw_pub = n.advertise<dw_listener::dwFiltered>("dw_filterer", 1000);

    ros::Subscriber dw_sub = n.subscribe("/dw_data", 1000, dwCallback);
    ros::Subscriber imu_sub = n.subscribe("/rc1/imu/data", 100, IMUcallback);

    double dt = 0.1;
    ros::Rate loop_rate(dt*100);


    //Kalman Filter Variables

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
            //ROS_INFO("%s ", nodeInfo.tagAddress.c_str());
            if (nodeID.compare(nodeInfo.tagAddress) == 0) {
                
                //Filter with gating 
            
                filteredData.gateFilter2(nodeInfo);
    
                //Kalman Filter

                kf.updateState({
                        static_cast<double>(IMUXaccel), //X accel
                        static_cast<double>(IMUYaccel)}, //Y accel
                        {
                        static_cast<double>(filteredData.XcoordGateFiltered), 
                        static_cast<double>(filteredData.YcoordGateFiltered)});
                
                vec estimate = kf.getEstimate();

                //SEND TO ROS MSG
                filteredData.XcoordKalmanFiltered = estimate(0);
                filteredData.YcoordKalmanFiltered = estimate(1);
                
                dw_pub.publish(filteredData.buildRosMsgFilter());
                filteredData.updateHistory(nodeInfo);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    } catch(std::exception& err) {
        ROS_INFO("ERROR will robinson! %s", err.what());
    }

    return 0;

}

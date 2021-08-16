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

//EKF STUFF ------------------------------------------------------------------------
// Process constants
const double m = 1.0;   // Mass in kg
const double g = 9.8;   // Gravitational accel.
const double d = 1.0;   // Length in m
const double b = 0.5;   // Friction coef. in 1/s

// Process function as standard function
vec processFunction(const vec& q, const vec& u) {
  vec q_pred = vec(2).zeros();

  q_pred(0) = q(0) + q(1) * system_dt;
  q_pred(1) = q(1) + (m * (u(0) - g) * d * sin(q(0)) - b * q(1)) * system_dt /
              (m * d * d);

  return q_pred;
}

// Output function as lambda
auto outputFunction = [](const vec& q)->vec{
  return {d * sin(q(0)), d * cos(q(0))}; };

// Process Jacobian as member function
struct ProcessJacobian {
  mat processJacobian(const vec& q, const vec& u) {
    double a11 = 1.0;
    double a12 = system_dt;
    double a21 = (u(0) - g) * cos(q(0)) * system_dt / d;
    double a22 = 1.0 - b * system_dt / (m * d * d);

    return { {a11, a12},
             {a21, a22} };
  }
}

// Output Jacobian as function object
struct outputJacobian {
  mat operator()(const vec& q) const {
    return { { d * cos(q(0)), 0.0},
             {-d * sin(q(0)), 0.0} };
  }
}
//EKF STUFF ----------------------------------------------------------------------------
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
    ExtendedKalmanFilter ekf(1, 2, 2);

    ekf.setProcessFunction(processFunction);

    ekf.setOutputFunction(outputFunction);

    ProcessJacobian s;
    using std::placeholders::_1;
    using std::placeholders::_2;
    ekf.setProcessJacobian(std::bind(&ProcessJacobian::processJacobian, &s, _1, _2));

    ekf.setOutputJacobian(outputJacobian());


    mat Q = {{0.001, 0.0}, {0.0, 0.001}};
    mat R = {{1.0, 0.0}, {0.0, 1.0}};

    ekf.setProcessCovariance(Q);
    ekf.setOutputCovariance(R);
    
     // Initial values (unknown by EKF)
    time[0] = 0.0;

    true_ang[0] = 1.5;
    true_vel[0] = 0.0;
    true_acc[0] = 0.0;

    measured_xy[0] = {0.0, 0.0};
    measured_ang[0] = 0.0;
    estimated_ang[0] = 0.0;
    believed_acc[0] = 0.0;


    try {

        dw_filtered filteredData; 
        while (ros::ok()){
            filteredData.XcoordGateFiltered = 0;
            filteredData.YcoordGateFiltered = 0;
            filteredData.XcoordKalmanFiltered = 0;
            filteredData.XcoordKalmanFiltered = 0;

            //Filter with gating 
           
            filteredData.gateFilter2(nodeInfo);
 
            //Kalman Filter

            ekf.updateState({
                    static_cast<double>(0), //X accel
                    static_cast<double>(0)}, //Y accel
                    {
                    static_cast<double>(filteredData.XcoordGateFiltered), 
                    static_cast<double>(filteredData.YcoordGateFiltered)});
            
            vec estimate = ekf.getEstimate();

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
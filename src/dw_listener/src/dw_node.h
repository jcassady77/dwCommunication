

#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "ros/ros.h"
#include <dw_listener/nodeData.h>

#define FILTER_SIZE 10
#define GATE_VALUE 5

/**
typedef struct
{
    String tagAddress;
    double rangeNum;
    double timeOfReception;
    double distance;
    double degrees;
    double Xcoord;
    double Ycoord;
    double clockOffset;
    double serviceData;
    double Xaccel;
    double Yaccel;
    double Zaccel;
    
} dw_data;
* */
class dw_data {
    public:
        //Raw Data from Serial
        std_msgs::String tagAddress;
        std_msgs::Int64 rangeNum;
        std_msgs::Int64 timeOfReception;
        std_msgs::Int64 distance;
        std_msgs::Int64 degrees;
        std_msgs::Int64 Xcoord;
        std_msgs::Int64 Ycoord;
        std_msgs::Int64 clockOffset;
        std_msgs::Int64 serviceData;
        std_msgs::Int64 Xaccel;
        std_msgs::Int64 Yaccel;
        std_msgs::Int64 Zaccel;

        //Interpreted Data
        std_msgs::Int64 xHistory[FILTER_SIZE];
        std_msgs::Int64 yHistory[FILTER_SIZE];
        std_msgs::Int64 xHistoryFilter[FILTER_SIZE];
        std_msgs::Int64 yHistoryFilter[FILTER_SIZE];

        std_msgs::Int64 XcoordGateFiltered;
        std_msgs::Int64 YcoordGateFiltered;
        std_msgs::Int64 XcoordKalmanFiltered;
        std_msgs::Int64 YcoordKalmanFiltered;

        //Methods
        dw_listener::nodeData buildRosMsg();
        void updateHistory();
        void gateFilter();
        void gateFilter2();
};

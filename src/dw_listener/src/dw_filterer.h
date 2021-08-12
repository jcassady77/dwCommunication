
#include "dw_node.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "ros/ros.h"
#include <dw_listener/nodeData.h>
#include <dw_listener/dwFiltered.h>

#define FILTER_SIZE 10
#define GATE_VALUE 5

class dw_filtered {
    public:
        double xHistory[FILTER_SIZE];
        double yHistory[FILTER_SIZE];
        double xHistoryFilter[FILTER_SIZE];
        double yHistoryFilter[FILTER_SIZE];


        double XcoordGateFiltered;
        double YcoordGateFiltered;
        double XcoordKalmanFiltered;
        double YcoordKalmanFiltered;
        
        //Methods
        dw_listener::dwFiltered buildRosMsgFilter();
        void updateHistory(dw_data);
        void gateFilter(dw_data);
        void gateFilter2(dw_data);
        void dwCallback(const dw_listener::nodeData::ConstPtr&);
};


#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "ros/ros.h"
#include <dw_listener/nodeData.h>

class dw_data {
    public:
        //Raw Data from Serial
        std::string tagAddress;
        double rangeNum;
        double timeOfReception;
        double distance;
        double degrees;
        double Xcoord;
        double Ycoord;
        double clockOffset;
        double serviceData;
        

        //Methods
        dw_listener::nodeData buildRosMsg();
        int countRightBrackets();

};

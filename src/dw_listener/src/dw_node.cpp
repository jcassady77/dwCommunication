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

class dw_data {
    public:
        string tagAddress;
        int64 rangeNum;
        int64 timeOfReception;
        int64 distance;
        int64 degrees;
        int64 Xcoord;
        int64 Ycoord;
        int64 clockOffset;
        int64 serviceData;
        int64 Xaccel;
        int64 Yaccel;
        int64 Zaccel;


}


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

//  CODE FROM DECAWAVE GUI

/**
* @brief motionFilter()
*        perfrom the motion-filter on the x and y inputs
* */
/**
void motionFilter(double* x, double* y, int i)
{
     tag_reports_t rp = _tagList.at(i);

     if (rp.filterHisIdx >= FILTER_SIZE)
     {
         rp.filterHisIdx = static_cast<int>(fmod(rp.filterHisIdx,FILTER_SIZE));
         rp.motionFilterReady = true;
     }

     rp.estXHis[rp.filterHisIdx] = *x;
     rp.estYHis[rp.filterHisIdx] = *y;

     rp.filterHisIdx = rp.filterHisIdx + 1;

     if (_motionFilterOn)
     {
         if(rp.motionFilterReady)
         {
             double tempX[FILTER_SIZE];
             memcpy(tempX, &rp.estXHis[0], sizeof(tempX));

             double tempY[FILTER_SIZE];
             memcpy(tempY, &rp.estYHis[0], sizeof(tempY));

             r95Sort(tempX,0,FILTER_SIZE-1);

             *x = (tempX[FILTER_SIZE/2] + tempX[FILTER_SIZE/2-1])/2;

             r95Sort(tempY,0,FILTER_SIZE-1);

             *y = (tempY[FILTER_SIZE/2] + tempY[FILTER_SIZE/2-1])/2;
         }

     }

     //update the list entry
     _tagList.replace(i, rp);
 }

/**
* @brief r95Sort()
*        R95 sort used by the motion filter function above
* */
/**
void r95Sort (double s[], int l, int r)
{
    int i,j;
    double x;
    if(l<r)
    {
        i = l;
        j = r;
        x = s[i];
        while(i<j)
        {
            while (i<j&&s[j]>x) j--;
            if (i<j) s[i++] = s[j];
            while (i<j&&s[i]<x) i++;
            if (i < j) s[j--] = s[i];
        }
        s[i] = x;
        r95Sort(s, l, i-1);
        r95Sort(s, i+1, r);
    }

}

**/
#include "dw_node.h"

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
#include <dw_listener/nodeData.h>
#include <cmath>





dw_listener::nodeData dw_data::buildRosMsg() {
    dw_listener::nodeData msg;
    msg.tagAddress = tagAddress.data;
    msg.rangeNum = rangeNum.data;
    msg.timeOfReception = timeOfReception.data;
    msg.distance = distance.data;
    msg.degrees = degrees.data;
    msg.Xcoord = Xcoord.data;
    msg.Ycoord = Ycoord.data;
    msg.clockOffset = clockOffset.data;
    msg.serviceData = serviceData.data;
    msg.Xaccel = Xaccel.data;
    msg.Yaccel = Yaccel.data;
    msg.Zaccel = Zaccel.data;
    return msg;
}

void dw_data::updateHistory() {
    int i;
    for (i = FILTER_SIZE-1; i > 0; i--)
    {
        this->xHistory[i] = this->xHistory[i-1];
        this->yHistory[i] = this->yHistory[i-1];
    }
    this->xHistory[0] = this->Xcoord;
    this->yHistory[0] = this->Ycoord;
}

void dw_data::gateFilter() {
    int i;
    //Find Means
    int meanX = 0;
    int meanY = 0;
    for (i = FILTER_SIZE-1; i > 0; i--)
    {
        meanX += xHistory[i].data;
        meanY += yHistory[i].data;
    }
    meanX /= FILTER_SIZE;
    meanY /= FILTER_SIZE;

    //find stdev
    int stdevX = 0;
    int stdevY = 0;
    for (i = FILTER_SIZE-1; i > 0; i--)
    {
        stdevX += abs(meanX - xHistory[i].data);
        stdevY += abs(meanY - yHistory[i].data);
    }
    stdevX /= FILTER_SIZE;
    stdevY /= FILTER_SIZE;

    //Implement Gate
    if (abs(Xcoord.data - xHistory[0].data) > (stdevX * 5))
    {
        this->Xcoord = this->xHistory[0];
    }
    if (abs(Ycoord.data - yHistory[0].data) > (stdevY * 5))
    {
        this->Ycoord = this->yHistory[0];
    }
    
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
        dw_data message;
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
                message.tagAddress.data = msg.data.substr(msg.data.find("a16")+6, (msg.data.find("R")-msg.data.find("a16")-9));
                message.rangeNum.data = stoi(msg.data.substr(msg.data.find("R")+3, (msg.data.find("T")-msg.data.find("R")-5)));
                message.timeOfReception.data = stoi(msg.data.substr(msg.data.find("T")+3, (msg.data.find("D")-msg.data.find("T")-5)));
                message.distance.data = stoi(msg.data.substr(msg.data.find("D")+3, (msg.data.find("P")-msg.data.find("D")-5)));
                message.degrees.data = stoi(msg.data.substr(msg.data.find("P")+3, (msg.data.find("Xcm")-msg.data.find("P")-5)));
                message.Xcoord.data = stoi(msg.data.substr(msg.data.find("Xcm")+5, (msg.data.find("Ycm")-msg.data.find("Xcm")-7)));
                message.Ycoord.data = stoi(msg.data.substr(msg.data.find("Ycm")+5, (msg.data.find("O")-msg.data.find("Ycm")-7)));
                message.clockOffset.data = stoi(msg.data.substr(msg.data.find("O")+3, (msg.data.find("V")-msg.data.find("O")-5)));
                message.serviceData.data = stoi(msg.data.substr(msg.data.find("V")+3, (msg.data.find("X\"")-msg.data.find("V")-5)));
                message.Xaccel.data = stoi(msg.data.substr(msg.data.find("X\"")+3, (msg.data.find("Y\"")-msg.data.find("X\"")-5)));
                message.Yaccel.data = stoi(msg.data.substr(msg.data.find("Y\"")+3, (msg.data.find("Z\"")-msg.data.find("Y\"")-5)));
                message.Zaccel.data = stoi(msg.data.substr(msg.data.find("Z\"")+3, (msg.data.find("}}")-msg.data.find("Z\"")-3)));

                
            } catch(std::exception& err) {
                ROS_INFO("Error while building dw_data:  %s", err.what());
            }
            //Filter with gateing 
            message.gateFilter();

            dw_pub.publish(message.buildRosMsg());
            message.updateHistory();
            

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
     //Need to generate a list of past X and Y positions
     tag_reports_t rp = _tagList.at(i);
    
    //once we have gotten a 'filter size' number of locations
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

     //update current pos
     _tagList.replace(i, rp);
 }
* */
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
* */


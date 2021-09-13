#include <dw_listener/nodeData.h>
#include <dw_listener/dwFiltered.h>

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// %EndTag(INCLUDES)%

double xPos = 0;
double yPos = 0;

double xPosRaw [5] = {0, 0, 0, 0, 0};
double yPosRaw [5] = {0, 0, 0, 0, 0};

void dataCallback(const dw_listener::dwFiltered::ConstPtr& msg)
{
  xPos = static_cast<double>(msg->XcoordKalmanFiltered) / 100.;
  yPos = static_cast<double>(msg->YcoordKalmanFiltered) / 100.;

  //xPos = xPos - 5;
  //yPos = yPos - 5;
}
void dataRawCallback(const dw_listener::nodeData::ConstPtr& msg)
{
  ROS_INFO("RAW CALLBACK");
  int index = atoi((msg->tagAddress).c_str()) - 1;

  xPosRaw[index] = static_cast<double>(msg->Xcoord) / 100.;
  yPosRaw[index] = static_cast<double>(msg->Ycoord) / 100.;

}
// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "nodeVis");
  ros::NodeHandle n;
  ros::Rate r(10);
  ros::Publisher marker_pub0 = n.advertise<visualization_msgs::Marker>("visualization_marker0", 1);
  ros::Publisher marker_pub1 = n.advertise<visualization_msgs::Marker>("visualization_marker1", 1);
  ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("visualization_marker2", 1);
  ros::Publisher marker_pub3 = n.advertise<visualization_msgs::Marker>("visualization_marker3", 1);
  ros::Publisher marker_pub4 = n.advertise<visualization_msgs::Marker>("visualization_marker4", 1);

//Ros subscribers
  ros::Subscriber sub = n.subscribe("/dw_filterer", 100, dataCallback);
  ros::Subscriber subRaw = n.subscribe("/dw_data", 100, dataRawCallback);
// %EndTag(INIT)%

  // Set our initial shape type to be a cube
// %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::SPHERE;
// %EndTag(SHAPE_INIT)%

// %Tag(MARKER_INIT)%
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/rc1_sensor_init";
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
    marker.ns = "nodeVis";
    marker.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    marker.type = shape;
// %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
// %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
// %EndTag(ACTION)%


    marker.pose.position.x = xPosRaw[0];
    marker.pose.position.y = yPosRaw[0];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
    while (marker_pub0.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
      
    }
    marker_pub0.publish(marker);

    marker.pose.position.x = xPosRaw[1];
    marker.pose.position.y = yPosRaw[1];

    marker_pub1.publish(marker);
    
    marker.pose.position.x = xPosRaw[2];
    marker.pose.position.y = yPosRaw[2];

    marker_pub2.publish(marker);

    marker.pose.position.x = xPosRaw[3];
    marker.pose.position.y = yPosRaw[3];

    marker_pub3.publish(marker);

    marker.pose.position.x = xPosRaw[4];
    marker.pose.position.y = yPosRaw[4];

    marker_pub4.publish(marker);
// %EndTag(PUBLISH)%
    // Cycle between different shapes
    ros::spinOnce();
// %Tag(SLEEP_END)%
    r.sleep();
	  
  }
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%

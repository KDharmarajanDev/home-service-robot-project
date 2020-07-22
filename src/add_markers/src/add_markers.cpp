#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

ros::Publisher marker_pub;

void putPickupMarker(ros::Publisher& marker_pub){
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  visualization_msgs::Marker startMarker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  startMarker.header.frame_id = "/map";
  startMarker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  startMarker.ns = "add_markers";
  startMarker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  startMarker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  startMarker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  startMarker.pose.position.x = 1.0;
  startMarker.pose.position.y = 0;
  startMarker.pose.position.z = 0;
  startMarker.pose.orientation.x = 0.0;
  startMarker.pose.orientation.y = 0.0;
  startMarker.pose.orientation.z = 0.0;
  startMarker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  startMarker.scale.x = 0.2;
  startMarker.scale.y = 0.2;
  startMarker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  startMarker.color.r = 1.0f;
  startMarker.color.g = 1.0f;
  startMarker.color.b = 0.0f;
  startMarker.color.a = 1.0;

  startMarker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(startMarker);
  sleep(5);
    
  startMarker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(startMarker);
}

void putDropOffMarker(ros::Publisher& marker_pub){
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  visualization_msgs::Marker endMarker;
    
  endMarker.header.frame_id = "/map";
  endMarker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  endMarker.ns = "add_markers";
  endMarker.id = 1;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  endMarker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  endMarker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  endMarker.pose.position.x = 2.0;
  endMarker.pose.position.y = 2.0;
  endMarker.pose.position.z = 0;
  endMarker.pose.orientation.x = 0.0;
  endMarker.pose.orientation.y = 0.0;
  endMarker.pose.orientation.z = 0.0;
  endMarker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  endMarker.scale.x = 0.2;
  endMarker.scale.y = 0.2;
  endMarker.scale.z = 0.2;

 // Set the color -- be sure to set alpha to something non-zero!
 endMarker.color.r = 1.0f;
 endMarker.color.g = 1.0f;
 endMarker.color.b = 0.0f;
 endMarker.color.a = 1.0;
  
 endMarker.lifetime = ros::Duration();
 marker_pub.publish(endMarker);
}


void handleCallback(const std_msgs::Bool::ConstPtr& msg){
  if(msg->data){
    putPickupMarker(marker_pub);
  } else {
    putDropOffMarker(marker_pub);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  std::string isUsingNavigationString;
  n.getParam("usingNagivation",isUsingNavigationString);
  bool isSyncedWithNavigation = (isUsingNavigationString == "true");
  
  if(isSyncedWithNavigation){
  	//reached_pickup is a topic only published on when the robot reaches the pickup location or the dropoff location
  	ros::Subscriber navigation_sub = n.subscribe("reached_pickup",1000,handleCallback);
    ros::spin();
  } else {
    putPickupMarker(marker_pub);
    sleep(5);
    putDropOffMarker(marker_pub);
  }
 r.sleep();
}
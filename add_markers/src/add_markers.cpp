#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <visualization_msgs/Marker.h>

double robot_pose_x, robot_pose_y, distance;
double threshold = 0.2;
double pickup_x = 8.0, pickup_y = 0.0, dropoff_x = 8.0, dropoff_y = 4.0;

void getPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  robot_pose_x = msg->pose.pose.position.x;
  robot_pose_y = msg->pose.pose.position.y;
}  

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pickup_x;
  marker.pose.position.y = pickup_y;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  
  ros::Subscriber sub = n.subscribe("/amcl_pose", 100, getPoseCallback);
  
  int state = 0;

  while (ros::ok())
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    if(state == 0){
        distance = sqrt(pow(robot_pose_x - pickup_x, 2) + pow(robot_pose_y - pickup_y, 2));
       // ROS_INFO_STREAM("DISTANCE: " << distance);
        if(distance < threshold){
           ROS_INFO_ONCE("Robot has reached the pickup zone, so deleting the object");
           marker.action = visualization_msgs::Marker::DELETE;
           state = 1;
        }
        else{
            //ROS_INFO_ONCE("Robot is far away from pickup zone, so showing the object");
            marker.action = visualization_msgs::Marker::ADD;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = pickup_x;
            marker.pose.position.y = pickup_y;
        }
    }
    
    else if(state == 1){
        //Wait 5 seconds to simulate a pickup
    	ros::Duration(5).sleep();
    	state = 2;
    }
    
    else if(state == 2){
     	distance = sqrt(pow(robot_pose_x - dropoff_x, 2) + pow(robot_pose_y - dropoff_y, 2));
        
        if(distance > threshold){
           marker.action = visualization_msgs::Marker::DELETE;
           ROS_INFO_ONCE("Robot on its way to deliver the object");
        }
        else{
            ROS_INFO_ONCE("Robot has delivered the object");
            marker.action = visualization_msgs::Marker::ADD;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = dropoff_x;
            marker.pose.position.y = dropoff_y;
        }
    }
    
    marker_pub.publish(marker);    
    ros::spinOnce();
  }
}
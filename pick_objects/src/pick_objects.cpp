#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double pickup_x = 8.0, pickup_y = 0.0, dropoff_x = 8.0, dropoff_y = 4.0;

int main(int argc, char ** argv){
  ros::init(argc, argv, "pick_objects");
  
  MoveBaseClient ac("move_base", true);
  
  while(!ac.waitForServer(ros::Duration(5))){
    ROS_INFO("Waiting for the move_base action server to come up!");
  }
  
  move_base_msgs::MoveBaseGoal goal;
  
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = pickup_x;
  goal.target_pose.pose.position.y = pickup_y;
  goal.target_pose.pose.orientation.w = 1.0;
  
  ROS_INFO("Sending pick-up location");
  
  ac.sendGoal(goal);
  ac.waitForResult();
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The robot picked up the virtual object.");
    
    ros::Duration(5).sleep();
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = dropoff_x;
    goal.target_pose.pose.position.y = dropoff_y;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending drop-off location");
    ac.sendGoal(goal);

    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    	ROS_INFO("The robot dropped off the virtual object.");
    }
    else{
    	ROS_INFO("The robot was unable to dropped off the virtual object.");
    }
  }
  else{
    ROS_INFO("The robot was unable to pick-up the virtual object.");
  }
  return 0;
}

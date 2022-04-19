#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h> 

using namespace std;

int user_choice = 2; 

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

    // Connect to ROS
    ros::init(argc, argv, "simple_navigation_goals");
  
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
  
    // Wait for the action server to come up so that we can begin processing goals.
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal_start;
    move_base_msgs::MoveBaseGoal goal_end;
    
    goal_start.target_pose.header.frame_id = "map";
    goal_start.target_pose.header.stamp = ros::Time::now();
    
    goal_end.target_pose.header.frame_id = "map";
    goal_end.target_pose.header.stamp = ros::Time::now();
    
    cout << "\nPlease choose the starting point."<<endl;
    
    boost::shared_ptr<geometry_msgs::PointStamped const> waypoint;
    waypoint = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/clicked_point");   
    ROS_INFO("pose is %f", waypoint->point.x);
    ROS_INFO("pose is %f", waypoint->point.y);
    
    goal_start.target_pose.pose.position.x = waypoint->point.x;
    goal_start.target_pose.pose.position.y = waypoint->point.y;
    goal_start.target_pose.pose.orientation.w = 1.0;
    
    cout << "\nPlease choose the end point."<<endl;
    
    waypoint = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/clicked_point");   
    ROS_INFO("pose is %f", waypoint->point.x);
    ROS_INFO("pose is %f", waypoint->point.y);
    
    goal_end.target_pose.pose.position.x = waypoint->point.x;
    goal_end.target_pose.pose.position.y = waypoint->point.y;
    goal_end.target_pose.pose.orientation.w = 1.0;
    
    for (int i = 0; i < 2; i++){
    	cout << "\nGoing to starting point." << endl;
    
    ac.sendGoalAndWait(goal_start, ros::Duration(180.0,0), ros::Duration(180.0,0));
 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("The robot has arrived at the starting location");
      
      cout << "\n Going to end point." << endl;
      
      ac.sendGoalAndWait(goal_end, ros::Duration(180.0,0), ros::Duration(180.0,0));
 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The robot has arrived at the goal location");
    else
      ROS_INFO("The robot may have failed to reach the goal location");
    }
    else
      ROS_INFO("The robot may have failed to reach the goal location");
        
    }
    
    
  return 0;
}

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

#include <visualization_msgs/Marker.h>

#include <list>

using namespace std;

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
    
    move_base_msgs::MoveBaseGoal goal;
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    boost::shared_ptr<geometry_msgs::PointStamped const> waypoint;
    std::list<boost::shared_ptr<geometry_msgs::PointStamped const>> points_list;
    
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "map";
    points_marker.header.stamp = ros::Time();
    
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    points_marker.ns = "points";
    points_marker.id =0;

    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    
    points_marker.type = visualization_msgs::Marker::POINTS;
    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    points_marker.scale.x = 1.0;
    points_marker.scale.y = 1.0;
    points_marker.scale.z = 1.0;
    
    // Points are green
    points_marker.color.g = 1.0f;
    points_marker.color.a = 1.0;
    
    cout << "\nPlease choose one point on map to start navigation."<<endl;
    
    while(true){
	waypoint = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/clicked_point");   
   	ROS_INFO("point.x is %f", waypoint->point.x);
   	ROS_INFO("point.y is %f", waypoint->point.y);
    	
    points_list.push_back(waypoint);
    	
    geometry_msgs::Point p;
	p.x = waypoint->point.x;
	p.y = waypoint->point.y;
	p.z = waypoint->point.z;

	points_marker.points.push_back(p);
	
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
		marker_pub.publish(points_marker);
    	
    	cout << "Continue? Y/N" << endl;
    	string input;
    	cin >> input;
    	if(input == "n" || input == "N") break;
    }
    
    if(points_list.size() != 0){
    	if(points_list.size() != 1){
    		points_list.push_back(points_list.front());
    	}
    	for(auto const& point: points_list){
    		ROS_INFO("goal is %f", point->point.x);
    		ROS_INFO("goal is %f", point->point.y);
    		
    	    goal.target_pose.pose.position.x = point->point.x;
    		goal.target_pose.pose.position.y = point->point.y;
   			goal.target_pose.pose.orientation.w = 1.0;
   			
   			cout << "\nGoing to next point." << endl;
    
    		ac.sendGoalAndWait(goal, ros::Duration(180.0,0), ros::Duration(180.0,0));
 
    		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      			ROS_INFO("The robot has arrived at the goal location");
      			}
    		else
      			ROS_INFO("The robot may have failed to reach the goal location");
    	}
    }
    
  return 0;
}

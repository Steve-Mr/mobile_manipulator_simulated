/*
 * Author: Automatic Addison
 * Date: July 20, 2021
 * ROS Version: ROS 1 - Noetic
 * Website: https://automaticaddison.com
 * This ROS node sends the robot goals to move to a particular location on 
 * a map.
 *
 * 1 = House 1
 * 2 = House 2
 * 3 = House 3
 * 4 = Post Office (Default)
 */
 
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
 
using namespace std;

int user_choice = 4; 

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

    // Ask the user where he wants the robot to go?
    cout << "\nWhere do you want the robot to go?" << endl;
    cout << "\n1 = House 1" << endl;
    cout << "2 = House 2" << endl;
    cout << "3 = House 3" << endl;
    cout << "4 = Post Office" << endl;
    cout << "\nEnter a number: ";
    cin >> user_choice;
 
    // Create a new goal to send to move_base 
    move_base_msgs::MoveBaseGoal goal;
 
    // Send a goal to the robot
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
 
    // Use map_server to load the map of the environment on the /map topic. 
    // Launch RViz and click the Publish Point button in RViz to 
    // display the coordinates to the /clicked_point topic.
    switch (user_choice) {
      case 1:
        cout << "\nGoal Location: House 1\n" << endl;
        goal.target_pose.pose.position.x = -15.04;
        goal.target_pose.pose.position.y = -7.42;
        goal.target_pose.pose.orientation.w = 1.0;
        break;
      case 2:
        cout << "\nGoal Location:House 2\n" << endl;
        goal.target_pose.pose.position.x = -14.25;
        goal.target_pose.pose.position.y = 20.02;
        goal.target_pose.pose.orientation.w = 1.0;
        break;
      case 3:
        cout << "\nGoal Location: House 3\n" << endl;
        goal.target_pose.pose.position.x = 7.35;
        goal.target_pose.pose.position.y = 20.17;
        goal.target_pose.pose.orientation.w = 1.0;
        break;
      case 4:
        cout << "\nGoal Location: Post Office\n" << endl;
        goal.target_pose.pose.position.x = 12.12;
        goal.target_pose.pose.position.y = -8.41;
        goal.target_pose.pose.orientation.w = 1.0;
        break;
      default:
        cout << "\nGoal Location: Post Office\n" << endl;
        goal.target_pose.pose.position.x = 12.12;
        goal.target_pose.pose.position.y = -8.41;
        goal.target_pose.pose.orientation.w = 1.0;
    }       
         
    ROS_INFO("Sending goal");
    
    // Set a timeout in seconds
    ac.sendGoalAndWait(goal, ros::Duration(180.0,0), ros::Duration(180.0,0));
 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The robot has arrived at the goal location");
    else
      ROS_INFO("The robot may have failed to reach the goal location");
   
  return 0;
}

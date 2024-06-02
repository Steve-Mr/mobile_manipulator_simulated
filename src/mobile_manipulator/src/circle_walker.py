#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import os

class CircleWalker:
    def __init__(self):
        rospy.init_node('circle_walker', anonymous=True)
        self.velocity_pub = rospy.Publisher('/robot_base_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.start_time = rospy.Time.now()
        print("start time: ", self.start_time.to_sec())
        self.circle_count = 0
        self.target_radius = 50.0  # 50 meters radius

    def walk(self):
        while not rospy.is_shutdown() and self.circle_count < 2:
            current_time = rospy.Time.now()
            elapsed_time = (current_time - self.start_time).to_sec()
            print(elapsed_time)
            angular_speed = 0.02  # Adjust angular speed as needed
            linear_speed = 4.0

            # Calculate the angle for current time
            angle = angular_speed * elapsed_time

            # Calculate the x, y coordinates on the circle
            x = self.target_radius * cos(angle)
            y = self.target_radius * sin(angle)

            # Create Twist message
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed

            # Publish Twist message
            self.velocity_pub.publish(twist)
            
            print(angle)

            # Check if one circle is complete
            if angle >= (2 * pi):  # 2 * pi (complete circle)
                self.start_time = rospy.Time.now()  # Reset start time
                self.circle_count += 1  # Increment circle count

            self.rate.sleep()

        # Stop the robot
        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        self.velocity_pub.publish(twist)
        data_logger.stop_and_save()  # Stop data logging

class DataLogger:
    def __init__(self):
        # rospy.init_node('data_logger', anonymous=True)

        self.base_truth_file = 'base_truth.txt'
        self.odom_file = 'odom.txt'
        self.odom_combined_file = 'odom_combined.txt'

        self.base_truth_sub = rospy.Subscriber(
            '/base_truth_odom', Odometry, self.base_truth_callback)
        self.odom_sub = rospy.Subscriber(
            '/robot_base_velocity_controller/odom', Odometry, self.odom_callback)
        self.odom_combined_sub = rospy.Subscriber(
            '/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.odom_combined_callback)

        # Clear existing files
        for file_name in [self.base_truth_file, self.odom_file, self.odom_combined_file]:
            if os.path.exists(file_name):
                os.remove(file_name)

    def base_truth_callback(self, msg):
        pose = msg.pose.pose
        self.write_to_file(self.base_truth_file, pose)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.write_to_file(self.odom_file, pose)

    def odom_combined_callback(self, msg):
        pose = msg.pose.pose
        self.write_to_file(self.odom_combined_file, pose)

    def write_to_file(self, file_path, pose):
        with open(file_path, 'a') as f:
            f.write(f"{pose.position.x} {pose.position.y} {pose.position.z}\n")
            
    def stop_and_save(self):
        self.base_truth_sub.unregister()
        self.odom_sub.unregister()
        self.odom_combined_sub.unregister()
        rospy.signal_shutdown("Recording stopped")

if __name__ == '__main__':
    try:
        walker = CircleWalker()
        data_logger = DataLogger()
        walker.walk()
    except rospy.ROSInterruptException:
        pass

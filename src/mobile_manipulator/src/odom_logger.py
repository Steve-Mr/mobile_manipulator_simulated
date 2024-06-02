#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
import os


class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger', anonymous=True)

        self.base_truth_file = 'base_truth.txt'
        self.odom_file = 'odom.txt'
        self.odom_combined_file = 'odom_combined.txt'
        self.odom_combined_file2 = 'odom_combined2.txt'

        self.base_truth_sub = rospy.Subscriber(
            '/base_truth_odom', Odometry, self.base_truth_callback)
        self.odom_sub = rospy.Subscriber(
            '/robot_base_velocity_controller/odom', Odometry, self.odom_callback)
        self.odom_combined_sub = rospy.Subscriber(
            '/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.odom_combined_callback)
        self.odom_combined_sub2 = rospy.Subscriber(
            '/robot_pose_ekf2/odom_combined', PoseWithCovarianceStamped, self.odom_combined_callback2)

        # Clear existing files
        for file_name in [self.base_truth_file, self.odom_file, self.odom_combined_file, self.odom_combined_file2]:
            if os.path.exists(file_name):
                os.remove(file_name)
                
        # self.rate = rospy.Rate(10)  # Record data at 10 Hz

    def base_truth_callback(self, msg):
        pose = msg.pose.pose
        self.write_to_file(self.base_truth_file, pose)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.write_to_file(self.odom_file, pose)

    def odom_combined_callback(self, msg):
        pose = msg.pose.pose
        self.write_to_file(self.odom_combined_file, pose)
    
    def odom_combined_callback2(self, msg):
        pose = msg.pose.pose
        self.write_to_file(self.odom_combined_file2, pose)
    
    def write_to_file(self, file_path, pose):
        with open(file_path, 'a') as f:
            f.write(f"{pose.position.x} {pose.position.y} {pose.position.z}\n")
            
    def stop_and_save(self):
        self.base_truth_sub.unregister()
        self.odom_sub.unregister()
        self.odom_combined_sub.unregister()
        self.odom_combined_sub2.unregister()
        rospy.signal_shutdown("Recording stopped")


if __name__ == '__main__':
    try:
        logger = DataLogger()
        rate = rospy.Rate(10)  # 设置频率为每秒1次

        while not rospy.is_shutdown():
            rate.sleep()  # 等待直到达到指定的频率
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass

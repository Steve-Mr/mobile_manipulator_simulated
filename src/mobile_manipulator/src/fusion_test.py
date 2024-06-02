#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import pi
from odom_logger import DataLogger

class RobotWalker:
    def __init__(self):
        rospy.init_node('robot_walker', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/robot_base_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.linear_speed = 0.5  # 0.5 m/s
        self.angular_speed = 0.5  # 0.5 rad/s

    def move_robot(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0

        # Move forward 10 meters
        distance = 0.0
        while distance < 10.0 and not rospy.is_shutdown():
            self.velocity_publisher.publish(twist)
            self.rate.sleep()
            distance += self.linear_speed / 10.0

        # Stop robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)

    def turn_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed

        # Turn left 120 degrees
        turn_angle = 0.0
        while turn_angle < (pi / 3) and not rospy.is_shutdown():
            self.velocity_publisher.publish(twist)
            self.rate.sleep()
            turn_angle += self.angular_speed / 10.0

        # Stop robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)

    def run(self):
        # Perform 36 turns
        # for _ in range(36):
        for _ in range(6):
            self.move_robot()
            rospy.sleep(1)  # Pause for 1 second before turning
            self.turn_robot()
            rospy.sleep(1)  # Pause for 1 second before next movement

if __name__ == '__main__':
    try:
        logger = DataLogger()
        robot_walker = RobotWalker()
        robot_walker.run()
    except rospy.ROSInterruptException:
        pass

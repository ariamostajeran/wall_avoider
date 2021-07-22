#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(msg):
    # If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
    if msg.ranges[0] > 1:
        move.linear.x = 0.5
        move.angular.z = 0.0

    # If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will stop
    print("Number of ranges: ", len(msg.ranges))
    print("Reading at position 0:", msg.ranges[0])
    if msg.ranges[0] < 1:
        move.linear.x = 0.0
        move.angular.z = 0.5

    pub.publish(move)


rospy.init_node('rotw5_node')
sub = rospy.Subscriber('/scan', LaserScan, callback)  # We subscribe to the laser's topic
pub = rospy.Publisher('/cmd_vel', Twist)
move = Twist()

rospy.spin()

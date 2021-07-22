#! /usr/bin/env python3
import math
import matplotlib.pyplot as plt
import numpy as np
import time
import rospy
from sensor_msgs.msg import LaserScan
import random
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
import tf
from nav_msgs.msg import Odometry


class WallDetector:
    def __init__(self):
        rospy.init_node('random_velocity')
        rospy.loginfo(
            "CTRL + C to stop the turtlebot")
        # rospy.on_shutdown(self.shutdown)
        sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.vel_pub = rospy.Publisher('/change', LaserScan, queue_size=1)
        self.average_error = 0

    def callback_laser(self, data_laser):
        direction = []
        fdist_to_wall = min(min(data_laser.ranges[0:15]), min(data_laser.ranges[345:360]))
        # fdist_to_wall1 = min(data_laser.ranges[345:360])
        # fdist_to_wall = min(fdist_to_wall1, fdist_to_wall1)
        print("FDIST TO WALL: ", fdist_to_wall)
        print("FRONT DIST : ", fdist_to_wall)
        mindist = min(data_laser.ranges)
        rdist_to_wall = min(data_laser.ranges[82:98])
        ldist_to_wall = min(data_laser.ranges[262:278])
        direction = [fdist_to_wall, mindist, rdist_to_wall, ldist_to_wall]
        self.vel_pub.publish(direction)



if __name__ == '__main__':
    try:
        generator = WallDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

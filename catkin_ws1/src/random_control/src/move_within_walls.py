#! /usr/bin/env python3
import math
import matplotlib.pyplot as plt
import numpy as np

import wall_detector
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


class RndVelocityGen:
    def __init__(self):
        rospy.init_node('random_velocity')
        rospy.loginfo(
            "CTRL + C to stop the turtlebot")
        # rospy.on_shutdown(self.shutdown)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.laser_Subscriber = rospy.Subscriber('/change', LaserScan, self.wall_callback)
        self.pose = Pose()
        self.start_time = time.time()
        self.coke_can = Pose()
        self.coke_can.x, self.coke_can.y = -8.3, -8.3
        self.coke_counter = 0
        self.plot_x = []
        self.plot_y = []
        self.vel = Twist()
        self.vel.linear.x = 0.5  # m/s
        self.vel.angular.z = 0.5  # rad/s
        self.max_interval = 10
        self.crashes = 0
        self.start_time = rospy.get_rostime().secs
        self.tmp = 0
        self.i_error = 0
        self.counter = 1
        self.vel_cal = 0
        self.average_error = 0
        self.directions = []
    def self_dist(self, pos1):
        return math.sqrt(((pos1.x - self.pose.x) ** 2) + ((pos1.y - self.pose.y) ** 2))

    # def wall_callback(self, msg):
    #     self.directions = msg

    def set_vel(self):

        while not rospy.is_shutdown():
            data_odom = None
            while data_odom is None:
                try:
                    data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
                    # data_twist = rospy.wait_for_message("/change", Twist, timeout=1)
                    self.pose.x = data_odom.pose.pose.position.x
                    self.pose.y = data_odom.pose.pose.position.y
                    quaternion = (data_odom.pose.pose.orientation.x, data_odom.pose.pose.orientation.y,
                                  data_odom.pose.pose.orientation.z, data_odom.pose.pose.orientation.w)

                    (roll, pitch, theta) = tf.transformations.euler_from_quaternion(quaternion)
                    # print("THETA = ", theta)
                    self.pose.theta = theta
                    # print("calculated theta = ", math.atan2(self.pose.x, self.pose.y))
                    # print("DATA_ODOM : ", data_odom.pose.pose.position)
                    # print("DATA_ODOM ANGLE :", self.pose.theta)
                except:
                    rospy.loginfo("CANT FIND ODOM")

            data_laser = None
            fdist_to_wall = 100
            rdist_to_wall = 100
            ldist_to_wall = 100
            mindist = 100
            # fdist_to_wall, mindist, rdist_to_wall, ldist_to_wall = self.directions
            while data_laser is None:
                try:
                    data_laser = rospy.wait_for_message("/scan", LaserScan, timeout=1)
                    # print("DATA RANGE", len(data_laser.ranges), "DATA LASER : ", data_laser.ranges)
                    fdist_to_wall = min(min(data_laser.ranges[0:15]), min(data_laser.ranges[345:360]))
                    # fdist_to_wall1 = min(data_laser.ranges[345:360])
                    # fdist_to_wall = min(fdist_to_wall1, fdist_to_wall1)
                    # print("FDIST TO WALL: ", fdist_to_wall)
                    # print("FRONT DIST : ", fdist_to_wall)
                    mindist = min(data_laser.ranges)
                    rdist_to_wall = min(data_laser.ranges[82:98])
                    ldist_to_wall = min(data_laser.ranges[262:278])
                except:
                    # print("Unexpected error:", e)
                    rospy.loginfo("CANT FIND LASER")

            if rospy.is_shutdown():
                self.shutdown()
                break
            point = Pose()

            x_forward = 0.7
            p_constant = 1.6
            safe_dist = 1.6
            z_counterclock = 0
            self.plot_x.append(self.pose.x)
            self.plot_y.append(self.pose.y)
            if self.self_dist(self.coke_can) < 2:
                self.tmp = True
            elif self.tmp and self.self_dist(self.coke_can) >= 2:
                self.tmp = False
                self.coke_counter += 1
            if fdist_to_wall < safe_dist:
                z_counterclock = (safe_dist - fdist_to_wall) * 1.1
                print('FDIST FROM WALL', fdist_to_wall)

            # elif rdist_to_wall < safe_dist - 1.2:
            #     z_counterclock = (safe_dist - rdist_to_wall - 1.2) * -0.55
            # elif ldist_to_wall < safe_dist - 1.2:
            #     z_counterclock = (safe_dist - ldist_to_wall - 1.2) * 0.55
            else:
                if random.randint(1, 25) == 1:
                    z_counterclock = 6
                    if random.randint(1, 3) == 1:
                        z_counterclock *= -1
                    print("RANDOM MOVE")

            if fdist_to_wall < safe_dist + 1.2:
                x_forward -= (safe_dist + 0.8 - fdist_to_wall) * 0.2

            if rdist_to_wall < ldist_to_wall and rdist_to_wall < 2.5:
                z_counterclock *= -1
            print("ANGULAR Result : ", z_counterclock)
            # print('DIST TO FRON WALL', fdist_to_wall)
            print("SPEED : ", x_forward)
            print("PASSED COKE COUNT : ", self.coke_counter)
            # print("CRASHES : ", self.crashes)
            if mindist < 0.05:
                self.crashes += 1
            self.vel.linear.x = x_forward
            self.vel.angular.z = z_counterclock
            # print("SELF = ", self.pose)
            self.vel_cal += x_forward
            self.counter += 1
            print("AVERAGE SPEED :", self.vel_cal / self.counter)
            self.vel_pub.publish(self.vel)
            now = rospy.get_rostime()
            print("Time passed: ", now.secs - 316)
            next = 0.05
            # rospy.loginfo("Twist: [%5.3f, %5.3f], next change in %i secs - ", self.vel.linear.x, self.vel.angular.z,
            #               next)
            plt.plot(self.plot_x, self.plot_y)
            plt.plot(-8.3, -8.3, 'ro')

            plt.legend(["Dataset 1", "Dataset 2"])
            plt.savefig("wall_move.pdf")

            rospy.sleep(next)

            # self.shutdown()

    def shutdown(self):
        print("Shutdown!")
        plt.plot(self.plot_x, self.plot_y)
        plt.plot(-8.3, -8.3, 'ro')

        plt.legend(["Dataset 1", "Dataset 2"])
        plt.savefig("wall_move.pdf")

        rospy.loginfo("Stop TurtleBot")

        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)
        return


if __name__ == '__main__':
    try:
        generator = RndVelocityGen()
        generator.set_vel()

    except rospy.ROSInterruptException:
        pass

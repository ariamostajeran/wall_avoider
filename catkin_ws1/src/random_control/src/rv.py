#! /usr/bin/env python3
import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
import random
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
        rospy.on_shutdown(self.shutdown)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.pose_Subscriber = rospy.Subscriber('/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.plot_x = []
        self.plot_y = []
        self.vel = Twist()
        self.vel.linear.x = 0.5  # m/s
        self.vel.angular.z = 0.5  # rad/s
        self.max_interval = 10
        self.start_time = rospy.get_rostime().secs
        self.tmp = 0
        self.i_error = 0
        self.counter = 0

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
                    print("THETA = ", theta)
                    self.pose.theta = theta
                    # print("calculated theta = ", math.atan2(self.pose.x, self.pose.y))
                    # print("DATA_ODOM : ", data_odom.pose.pose.position)
                    # print("DATA_ODOM ANGLE :", self.pose.theta)
                except:
                    rospy.loginfo("CANT FIND ODOM")

            if rospy.is_shutdown():
                self.shutdown()
                break
            point = Pose()

            safe_dist = 0.18

            x_forward = 0.47
            p_constant = 1.6
            z_counterclock = 0
            # if abs(angle_to_goal) > 0.05:
            #     z_counterclock = p_constant * (angle_to_goal) + i_constant * (self.i_error)

            print("ANGULAR Result : ", z_counterclock)

            self.vel.linear.x = x_forward
            print("linear speed", self.vel.linear.x)
            # print("ANGULAR VEL : ", self.vel.angular)
            self.vel.angular.z = z_counterclock
            print("counter ", self.counter)
            print("SELF = ", self.pose)
            print("LOCATION Follower", self.tmp)

            self.vel_pub.publish(self.vel)
            now = rospy.get_rostime()
            print("Time now: ", now.secs)
            next = 0.05
            rospy.loginfo("Twist: [%5.3f, %5.3f], next change in %i secs - ", self.vel.linear.x, self.vel.angular.z,
                          next)
            # print("plot x", self.plot_x)
            # plt.plot(self.plot_x, self.plot_y)
            # plt.plot(self.locations[0], self.locations[1])
            # plt.legend(["Dataset 1", "Dataset 2"])
            # plt.savefig("plots_1.pdf")

            rospy.sleep(next)
        # rospy.spin()

            self.shutdown()

    def shutdown(self):
        print("Shutdown!")

        rospy.loginfo("Stop TurtleBot")

        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        generator = RndVelocityGen()
        generator.set_vel()

    except rospy.ROSInterruptException:
        pass
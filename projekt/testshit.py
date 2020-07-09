#!/usr/bin/env python

import rospy
import numpy as np
from math import atan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

CHEESE_INTEREST_FACTOR = 0.5
CONSTANT_CAT_SPEED = 0.22
MAX_TURNING_FACTOR = 1


class Cat:
    def __init__(self):
        self.x_cheese = rospy.get_param('cheese_pos_x')
        self.y_cheese = rospy.get_param('cheese_pos_y')
        self.x_mouse = None
        self.y_mouse = None
        self.x_cat = None
        self.y_cat = None
        self.attraction_point_x = None
        self.attraction_point_y = None
        self.lin_vel_x = None
        self.lin_vel_ang = None
        self.sensor_angles = None
        self.sensor_ranges = None
        self.orientation = None

        rospy.Subscriber("cat/odom", Odometry, self.cat_odom_callback)
        rospy.Subscriber("mouse/odom", Odometry, self.mouse_odom_callback)
        rospy.Subscriber("cat/scan", LaserScan, self.laser_callback)

        self.cat_publisher = rospy.Publisher("cat/cmd_vel", Twist, queue_size=10)
        rospy.spin()

    def cat_odom_callback(self, odom):
        self.x_cat = odom.pose.pose.position.x
        self.y_cat = odom.pose.pose.position.y
        self.lin_vel_x = odom.twist.twist.linear.x
        self.lin_vel_ang = odom.twist.twist.angular.z
        self.orientation = odom.pose.pose.orientation.z

    def mouse_odom_callback(self, odom):
        self.x_mouse = odom.pose.pose.position.x
        self.y_mouse = odom.pose.pose.position.y

        # calc force

        self.calc_attraction_point()

        theta = atan((self.attraction_point_x-self.x_cat)/(self.attraction_point_y-self.y_cat))

        herp = abs(self.orientation - theta)
        if herp > 0.8:
            herp = 0.8

        adjust = 0
        if theta == self.orientation:
            adjust = 0
        elif theta > self.orientation:
            adjust = herp
        elif theta < self.orientation:
            adjust = -1 * herp


        #adjust = (self.lin_vel_ang - theta) * MAX_TURNING_FACTOR

        #if adjust > 0.8:
            #adjust = 0.8


        print(herp)
        velocity_adjustment = Twist()
        velocity_adjustment.linear.x = CONSTANT_CAT_SPEED
        velocity_adjustment.angular.z = adjust
        self.cat_publisher.publish(velocity_adjustment)

    def laser_callback(self, current_laser_scan):
        self.sensor_angles = np.arange(current_laser_scan.angle_min,
                                       current_laser_scan.angle_max + current_laser_scan.angle_increment,
                                       current_laser_scan.angle_increment)
        self.sensor_ranges = np.array(current_laser_scan.ranges)

    def calc_attraction_point(self):
        if self.x_mouse and self.y_mouse:
            self.attraction_point_x = (self.x_mouse - self.x_cheese) * CHEESE_INTEREST_FACTOR
            self.attraction_point_y = (self.y_mouse - self.y_cheese) * CHEESE_INTEREST_FACTOR


if __name__ == '__main__':
    rospy.init_node('cat', anonymous=True)
    try:
        node = Cat()
    except rospy.ROSInterruptException:
        pass

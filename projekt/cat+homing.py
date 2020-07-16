#!/usr/bin/env python

import rospy
import numpy as np
import math as m
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

CHEESE_INTEREST_FACTOR = 0.5
CONSTANT_CAT_SPEED = 0.22
MAX_TURNING_FACTOR = 1


class Cat:
    def __init__(self):
        self.x_cheese = rospy.get_param('cheese_pos_x')
        self.y_cheese = rospy.get_param('cheese_pos_y')
        self.x_mouse = 0
        self.y_mouse = 0
        self.x_cat = 0
        self.y_cat = 0
        self.attraction_point_x = None
        self.attraction_point_y = None
        self.lin_vel_x = 0.2
        self.lin_vel_ang = None
        self.sensor_angles = None
        self.sensor_ranges = None

        # fuer homing
        self.rho=1
        self.alpha=1
        self.phi_cat=1

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
        quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w,]
        euler = euler_from_quaternion(quat)
        self.phi_cat = euler[2]

        #self.update_polar()
        #self.homing()

    def mouse_odom_callback(self, odom):
        self.x_mouse = odom.pose.pose.position.x
        self.y_mouse = odom.pose.pose.position.y

        # calc force
        self.calc_attraction_point()
        self.update_polar()
        self.homing()

    def laser_callback(self, current_laser_scan):
        self.sensor_angles = np.arange(current_laser_scan.angle_min,
                                       current_laser_scan.angle_max + current_laser_scan.angle_increment,
                                       current_laser_scan.angle_increment)
        self.sensor_ranges = np.array(current_laser_scan.ranges)

        self.update_polar()
        self.homing()
        #force = self.calculate_force(self.sensor_angles, self.sensor_ranges)
        #out = Twist()
        #out.linear.x  = force[0]
        #out.angular.z = force[1]
        #self.cat_publisher.publish(out)

    def calculate_force(self,sonar_angles,sonar_ranges):

        force = np.zeros(2)

        for i in range(len(sonar_ranges)):
            if (sonar_ranges[i] < 0.7):
                force[0] += -m.cos(sonar_angles[i])*(0.7-sonar_ranges[i])
            if (sonar_ranges[i] < 0.8):
                force[1] += -m.sin(sonar_angles[i])*(0.8-sonar_ranges[i])

        force[0] = (force[0]/50) + 0.8
        if (force[0] > 0.8):
            force[0] = 0.8

        force[1] /= 24
        if ((force[1] < 0.1) and (force[0] < 0.2)):
            force[1] *= 2

        return force

    def calc_attraction_point(self):
        if self.x_mouse and self.y_mouse:
            self.attraction_point_x = (self.x_mouse - self.x_cheese) * CHEESE_INTEREST_FACTOR
            self.attraction_point_y = (self.y_mouse - self.y_cheese) * CHEESE_INTEREST_FACTOR

    def update_polar(self):
        delta_x = self.x_mouse - self.x_cat
        delta_y = self.y_mouse - self.y_cat
        # fuer homing
        self.roh = m.sqrt(delta_x**2 + delta_y**2) # Distanz zum ziel
        self.alpha = m.atan2(delta_y,delta_x) - self.phi_cat # Winkel zum ziel

    def homing(self):
        k_rho_0 = 0.1
        k_alpha_0 = 0.3

        force = self.calculate_force(self.sensor_angles, self.sensor_ranges)
        ##Copx paste
        if np.abs(self.alpha) < 0.3:
            if np.abs(self.rho) < 0.1:
                k_rho_0 = 1 * k_rho_0
                k_alpha_0 = 1 * k_alpha_0
            else:
                k_rho_0 = 1 * k_rho_0
                k_alpha_0 = 2 * k_alpha_0
        else:
            if np.abs(self.rho) < 0.1:
                k_rho_0 = 2 * k_rho_0
                k_alpha_0 = 2 * k_alpha_0
            else:
                k_rho_0 = 0 * k_rho_0
                k_alpha_0 = 4 * k_alpha_0

        out = Twist()
        out.linear.x =  k_rho_0 * self.rho # + force[0]
        out.angular.z =  k_alpha_0 * self.alpha# + force[1]
        self.cat_publisher.publish(out)

if __name__ == '__main__':

    rospy.init_node('cat', anonymous=True)

    try:
        node = Cat()
    except rospy.ROSInterruptException:
        pass

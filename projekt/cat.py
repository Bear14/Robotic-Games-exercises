#!/usr/bin/env python

import rospy, math
from nav_msgs.msg import Odometry




global x_cheese, y_cheese, x_cat, y_cat, x_mouse, y_mouse

def cat_odom_callback(odom):
    global x_mouse, y_mouse, x_cat, y_cat
    x_cat = odom.pose.pose.position.x
    y_cat = odom.pose.pose.position.y

def mouse_odom_callback(odom):
    global x_mouse, y_mouse, x_cat, y_cat
    x_mouse = odom.pose.pose.position.x
    y_mouse = odom.pose.pose.position.y


if __name__ == '__main__':

    global x_cheese, y_cheese, x_cat, y_cat, x_mouse, y_mouse

    x_cheese = rospy.get_param('cheese_pos_x')
    y_cheese = rospy.get_param('cheese_pos_y')

    #x_cat = rospy.get_param('cat_start_pos_x')
    #y_cat = rospy.get_param('cat_start_pos_y')

    #x_mouse = rospy.get_param('mouse_start_pos_x')
    #y_mouse = rospy.get_param('mouse_start_pos_y')

    rospy.init_node('cat', anonymous=True)
    rate = rospy.Rate(10)
    #rospy.Subscriber("cat/odom", Odometry, cat_odom_callback)
    #rospy.Subscriber("mouse/odom", Odometry, mouse_odom_callback)


    while not rospy.is_shutdown():
        rospy.loginfo(x_cheese, y_cheese)
        rate.sleep()

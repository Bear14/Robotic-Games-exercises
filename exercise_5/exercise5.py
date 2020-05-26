#!/usr/bin/env python
import numpy as np
import rospy
import copy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry



def update_state(state,input_val):
    speed         = 0.4 
    dt            = 1.0/iterations
    current_state = copy.deepcopy(state)

    # Aufgabe 2 hier implementieren

    return current_state

def reward(state):
    value = 0

    # Aufgabe 3 hier implementieren

    return value


class mini_agent:

    def __init__(self):
        self.scan_steps = 7
        self.game_state       = np.ones(self.scan_steps)
        self.strategy_choices = np.linspace(-np.pi / 4, np.pi / 4, self.scan_steps)  #diskretisierter Entscheidungsraum

        rospy.Subscriber("\odom",Odometry,self.odom_callback)

        # Aufgabe 1 subscriber hier implementieren! callback soll klassenfunktion sein
        rospy.Subscriber("\scan", LaserScan, self.scan_callback)


        pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)

        while not rospy.is_shutdown():
            out           = Twist()
            out.linear.x  = 0.4 #standart forward speed 
            current_state = copy.copy(self.game_state)
            move_value    = [reward(update_state(current_state,move)) for move in self.strategy_choices]
            out.angular.z = self.strategy_choices[np.argmax(move_value)]
            pub.publish(out)

    def scan_callback(self, data):
        keys = np.linspace(315, 405, self.scan_steps)
        for i in range(len(keys)):
            keys[i] = keys[i] % 360

        for j in range(len(keys)):
            self.game_state[j] = data.ranges[keys[j]]


    def odom_callback(self,data):
        orientation = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])[2]
        x_pos       = data.pose.pose.position.x
        y_pos       = data.pose.pose.position.y
        self.game_state[0]  = x_pos
        self.game_state[1]  = y_pos
        self.game_state[2]  = orientation



if __name__ == '__main__':
    rospy.init_node("Agent")
    try:
        node = mini_agent()
    except rospy.ROSInterruptException:
        pass


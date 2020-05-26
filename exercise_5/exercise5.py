#!/usr/bin/env python
import numpy as np
import rospy
import copy
import math
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry



def update_state(state,input_val):
    speed         = 0.4 
    dt            = 1.0/iterations
    current_state = copy.deepcopy(state)

    # Aufgabe 2 hier implementieren
    
    current_state[0] = state[0] + speed*math.cos(input_val)
    current_state[1] = state[1] + speed*math.sin(input_val)
    current_state[2] = state[2] + input_val
    if(current_state[2] >= math.pi):
        current_state[2] *= -1
    elif(current_state[2] <= -math.pi):
        current_state[2] *= -1
    dist = math.sqrt((current_state[0] - state[0])**2 + (current_state[1] - state[1])**2)

    for i in range(3, len(current_state)):
        angle = state[i][1]
        if (angle > 180):
            angle = 360 - angle
        current_state[i][0] = math.sqrt(dist**2 + state[i][0]**2 - (2*dist*state[i][0] * math.cos(angle)))

    return current_state

def reward(state):
    value = 0

    # Aufgabe 3 hier implementieren

    return value


class mini_agent:

    def __init__(self):
        self.scan_steps       = 7
        self.game_state       = np.ones(self.scan_steps + 3)
        self.strategy_choices = np.linspace(-0.2,0.2,10)  #diskretisierter Entscheidungsraum
        #-np.pi / 4, np.pi / 4, self.scan_steps
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

        for j in range(3, len(keys)+3):
            self.game_state[j] = np.array(data.ranges[keys[j-3]], keys[j-3])

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


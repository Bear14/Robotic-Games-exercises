#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist



'''
Die gesammte Kollisionsvermeidung ist in einer Klasse verpackt. Die aktuellen Geschwindigkeitsdaten werden durch den Callback des
Geschwindigkeits-Subscribers bereitgestellt. Die aktuelle Geschwindigkeit wird da fuer die Berechnung der neuen Sollgeschwindigkeit verwendet. 
Alternativ haetten hier auch globale Variabeln verwendet werden koennen. Diese Methode wird in der Community allerdings als
eleganter angesehen.
'''
class CollisionAvoidance:

    def __init__(self):

        self.current_lin_vel_x = 0.0
        self.current_ang_vel_z = 0.0


        '''
        Die momentane Geschwindigkeit des Roboters wird in der Simulation auf dem Topic odom gepublisht. 
        Hier wollen wir subscriben. Ebenfalls benoetigen wir die Daten des Laserscanners. 
        '''
        rospy.Subscriber("odom", Odometry, self.velocity_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        '''
        Das Ergebnis der Berechnung wird dem Roboter als Soll-Geschwindigkeit zurueckgegeben.
        '''
        self.col_avoid_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.spin()

    def velocity_callback(self, current_odometry):
        self.current_lin_vel_x = current_odometry.twist.twist.linear.x
        self.current_ang_vel_z = current_odometry.twist.twist.angular.z


    def laser_callback(self, current_laser_scan):
        sensor_angles = np.arange(current_laser_scan.angle_min, 
                          current_laser_scan.angle_max + current_laser_scan.angle_increment,
                          current_laser_scan.angle_increment)
        sensor_ranges = np.array(current_laser_scan.ranges)
        force = self.calculate_force(sensor_angles, sensor_ranges)
        velocity_adjustment = Twist()
        velocity_adjustment.linear.x = force[0]
        velocity_adjustment.angular.z = force[1]
        
        self.col_avoid_publisher.publish(velocity_adjustment)


    def calculate_force(self,sonar_angles,sonar_ranges):
    
        force = np.zeros(2)
        
        '''
        HIER KOMMT DER CODE HIN

        Das Ergebniss der Berechnungen soll in Form eines 2-dimensionalen
        Arrays zurueckgegeben werden.
        Die 1. Komponente entspricht dabei der Aenderung der Lineargeschwindigkeit
        und die 2. enstprechend der Aenderung der Winkelgeschwindigkeit.
        '''
        # sum the d's
        wall_dist = 5
        d_sum = np.zeros(2)
        for i in range(len(sonar_ranges)):
            phi = sonar_angles[i]
            distance = wall_dist-sonar_ranges[i]
            if distance < 0:
                distance = 0

            d = distance * np.array([np.sin(phi), np.cos(phi)])
            d_sum += d


        d_sum2 = np.abs(complex(d_sum[0],d_sum[1]))
        d_sum3 = np.angle(complex(d_sum[0],d_sum[1]))


        force = -1*np.array([d_sum2, d_sum3])

        #force = (force - 0.1) / (1 - 0.1)

        return force




if __name__ == '__main__':

    rospy.init_node("CollisionAvoidance")

    try:
        node = CollisionAvoidance()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import math


class Robot():

	def __init__(self, x, y, phi, width=0, height=0):

		self.pos_x = x
		self.pos_y = y
		self.orientation = phi
		self.trajectory = []
		
		self.width = 20
		self.height = 20
		
		
	def kin_update(self, v, omega, delta_t):
	
		#self.trajectory.append([self.pos_x, self.pos_y]) # add old position value to trajectory
		# TODO 4.1) implement kinematics

		kinematic_array = (np.matrix([[self.pos_x], [self.pos_y], [self.orientation]]) + np.matrix([[math.cos(self.orientation), 0], [math.sin(self.orientation), 0], [0, 1]]) * np.matrix([[v], [omega]]) * delta_t)
		self.pos_x, self.pos_y, self.orientation = kinematic_array.item(0), kinematic_array.item(1), kinematic_array.item(2)


	def distance(self, other_robot):
		return math.sqrt((self.pos_x - other_robot.pos_x) * (self.pos_x - other_robot.pos_x) + (self.pos_y - other_robot.pos_y) * (self.pos_y - other_robot.pos_y))

	def alignment_force(self, robots, radius):
		force = np.zeros(2)
		# TODO 4.2) implement behavior
		avg_dx = 0
		avg_dy = 0
		num_neighbours = 0
		for robot in robots:
			if self.distance(robot) < radius:
				avg_dx += robot.pos_x
				avg_dy += robot.pos_y
				num_neighbours += 1

		if num_neighbours:
			avg_dx = avg_dx / num_neighbours
			avg_dy = avg_dy / num_neighbours

			#avg_dx -= self.pos_x
			#avg_dy -= self.pos_y
			#buff = np.linalg.norm(np.array([avg_dx, avg_dy]))
			#if buff > 0:
			#	force[0] = avg_dx / buff
			#	force[1] = avg_dy / buff
			force[0] = avg_dx
			force[1] = avg_dy

		return force

	
	def cohesion_force(self, robots, radius): # center of mass
		force = np.zeros(2)
		# TODO 4.2) implement behavior

		center_x = 0
		center_y = 0
		num_neighbours = 0

		for robot in robots:
			if self.distance(robot) < radius:
				center_x += robot.pos_x
				center_y += robot.pos_y
				num_neighbours += 1

		if num_neighbours:
			center_x = center_x / num_neighbours
			center_y = center_y / num_neighbours

			force[0] = (center_x - self.pos_x)
			force[1] = (center_y - self.pos_y)

		return force

	
	def separation_force(self, robots, radius): # 
		force = np.zeros(2)
		# TODO 4.2) implement behavior
		min_distance = 1

		move_x = 0
		move_Y = 0
		for robot in robots:
			if not self == robot:
				if self.distance(robot) < min_distance:
					move_x += self.pos_x - robot.pos_y
					move_Y += self.pos_y - robot.pos_y


		force[0] = move_x
		force[1] = move_Y


		return force

		
	def behavior_update(self, robots):
		total_force = np.zeros(2)
		# TODO 4.2) play with values
		c1 = 5 * 1
		c2 = 1
		c3 = 1
		
		align = self.alignment_force(robots, 5)
		separation = self.separation_force(robots, 5)
		cohesion = self.cohesion_force(robots, 500)
		total_force = c1 * align + c2 * separation  + c3 * cohesion

           
		# Calc force (to absolut value and angle)
		total_force=np.array([np.sqrt(total_force[0]**2+total_force[1]**2),np.arctan2(total_force[0],total_force[1])])
		return total_force
	
	



	def edges(self):
		if self.pos_x > self.width:
			self.pos_x = 0
		elif self.pos_x < 0:
			self.pos_x = self.width

		if self.pos_y > self.height:
			self.pos_y = 0
		elif self.pos_y < 0:
			self.pos_y = self.height

	def show(self):
		for x in range(len(self.trajectory)):
			point = self.trajectory[x]
			plt.plot(point[0], point[1], color='grey', marker='o', markersize=2)
		plt.arrow(self.pos_x, self.pos_y, 0.2*math.cos(self.orientation), 0.2*math.sin(self.orientation),head_width=0.2, head_length=0.3, fc='k', ec='k')
		
	
		
		
		
	

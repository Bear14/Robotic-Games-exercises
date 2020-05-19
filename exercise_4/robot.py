#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import math


class Robot():

	def __init__(self, x, y, phi, width=0, height=0):

		self.pos_x = x
		self.pos_y = y
		self.orientation = phi
		self.trajectory = [[0, 0]]		# init value needed for alignment_force calculation
		
		self.width = 20
		self.height = 20
		
		
	def kin_update(self, v, omega, delta_t):
		if(len(self.trajectory)) > 5:		# make trajectory list to circular buffer to speed up the simulation
			self.trajectory.pop(0)
		self.trajectory.append([self.pos_x, self.pos_y])		# add old position value to trajectory
		# TODO 4.1) implement kinematics

		kinematic_array = (np.matrix([[self.pos_x], [self.pos_y], [self.orientation]]) + np.matrix([[math.cos(self.orientation), 0], [math.sin(self.orientation), 0], [0, 1]]) * np.matrix([[v], [omega]]) * delta_t)
		self.pos_x, self.pos_y, self.orientation = kinematic_array.item(0), kinematic_array.item(1), kinematic_array.item(2)

	def distance_to(self, other_robot):
		return math.sqrt((self.pos_x - other_robot.pos_x) * (self.pos_x - other_robot.pos_x) + (self.pos_y - other_robot.pos_y) * (self.pos_y - other_robot.pos_y))

	def alignment_force(self, robots, radius):
		force = np.zeros(2)
		# TODO 4.2) implement behavior
		avg_vel = np.zeros(2)
		num_neighbours = 0
		for robot in robots:
			if self.distance_to(robot) < radius:
				direction = np.array([1, 0])
				if robot.trajectory[len(robot.trajectory)-1][0]:
					tra_vec = robot.trajectory[len(robot.trajectory)-1]
					direction = np.array([robot.pos_x - tra_vec[0], robot.pos_y - tra_vec[1]])

				avg_vel += np.array([math.cos(robot.orientation), math.sin(robot.orientation)]) * np.linalg.norm(direction)
				num_neighbours += 1

		if num_neighbours:
			avg_vel /= num_neighbours

			force = avg_vel

		return force

	
	def cohesion_force(self, robots, radius): # center of mass
		force = np.zeros(2)
		# TODO 4.2) implement behavior

		num_neighbours = 0

		for robot in robots:
			if self.distance_to(robot) < radius:
				force[0] += robot.pos_x
				force[1] += robot.pos_y
				num_neighbours += 1

		if num_neighbours:
			force = force / num_neighbours

			force[0] = (force[0] - self.pos_x)
			force[1] = (force[1] - self.pos_y)

		return force

	
	def separation_force(self, robots, radius):
		force = np.zeros(2)
		# TODO 4.2) implement behavior
		min_distance = 30		# magic number O.o height + 10 but might need to be changed

		force[0] = 0
		force[1] = 0
		for robot in robots:
			if not self == robot:
				if self.distance_to(robot) < radius and self.distance_to(robot) < min_distance:
					force[0] += self.pos_x - robot.pos_y
					force[1] += self.pos_y - robot.pos_y

		return force

		
	def behavior_update(self, robots):
		total_force = np.zeros(2)
		# TODO 4.2) play with values
		c1 = 5
		c2 = 1
		c3 = 1
		
		align = self.alignment_force(robots, 5)
		separation = self.separation_force(robots, 5)
		cohesion = self.cohesion_force(robots, 500)
		total_force = c1 * align + c2 * separation + c3 * cohesion

           
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
		
	
		
		
		
	

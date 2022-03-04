#!/usr/env/bin python3

"""
ENPM661 Spring 2022: Planning for Autonomous Robots
Project 2: Dijkstra

Author(s):
Tanuj Thakkar (tanuj@umd.edu)
M. Engg Robotics
University of Maryland, College Park
"""

# Importing modules
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import cv2


class Map():

	def __init__(self, height, width):
		self.height = height
		self.width = width
		xx, yy = np.mgrid[0:height,0:width]
		self.occupancy_grid = np.zeros([self.height+4, self.width+4], dtype=bool)

	def generate_map(self):
		self.occupancy_grid[:2] = True
		self.occupancy_grid[:,:2] = True
		self.occupancy_grid[-2:] = True
		self.occupancy_grid[:,-2:] = True
		plt.plot(np.where(self.occupancy_grid==True)[0], np.where(self.occupancy_grid==True)[1], 'ks')
		plt.show()


class Circle():

	def __init__(self, x, y, radius):
		self.name = "Circle"
		self.x = x
		self.y = y
		self.radius = radius
		self.occupancy_grid = np.zeros([self.radius, self.radius], dtype=bool)

	def generate_circle(self):
		xx, yy = np.mgrid[0:self.radius*2,0:self.radius*2]
		self.occupancy_grid = np.logical_and(True, (((xx - self.x) ** 2 + (yy - self.y) ** 2) <= self.radius**2))
		# print(self.occupancy_grid.shape)
		# plt.plot(np.where(self.occupancy_grid==True)[0], np.where(self.occupancy_grid==True)[1], 'ks')
		# plt.show()

def main():
	m = Map(400,250)
	c = Circle(40, 40, 40)
	c.generate_circle()
	m.occupancy_grid[120:200,120:200] = c.occupancy_grid
	m.generate_map()

if __name__ == '__main__':
	main()
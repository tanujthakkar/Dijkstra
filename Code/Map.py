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

	def __init__(self, height: int, width: int) -> None:
		self.height = height
		self.width = width
		self.xx, self.yy = np.mgrid[0:height+4,0:width+4]
		self.occupancy_grid = np.zeros([self.height+4, self.width+4], dtype=int)

	def generate_map(self) -> None:
		c = Circle(300, 185, 40)
		circle = c.generate_circle(self.xx, self.yy)
		self.occupancy_grid += circle

		c = Circle(300, 185, 45)
		clearance = c.generate_circle(self.xx, self.yy)
		self.occupancy_grid += clearance

		self.occupancy_grid[:2] = -1
		self.occupancy_grid[:,:2] = -1
		self.occupancy_grid[-2:] = -1
		self.occupancy_grid[:,-2:] = -1

		plt.plot(np.where(self.occupancy_grid==-1)[0], np.where(self.occupancy_grid==-1)[1], 'ks')
		plt.plot(np.where(self.occupancy_grid==1)[0], np.where(self.occupancy_grid==1)[1], '.')
		plt.plot(np.where(self.occupancy_grid>1)[0], np.where(self.occupancy_grid>1)[1], '.k')
		plt.show()


class Circle():

	def __init__(self, x: int, y: int, radius: int) -> None:
		self.name = "Circle"
		self.x = x
		self.y = y
		self.radius = radius

	def generate_circle(self, xx, yy) -> np.array:
		occupancy_grid = np.logical_and(True, (((xx - self.x) ** 2 + (yy - self.y) ** 2) <= self.radius**2))
		occupancy_grid[occupancy_grid == True] = 1

		return occupancy_grid

def main():
	m = Map(400,250)
	m.generate_map()

if __name__ == '__main__':
	main()
#!/usr/env/bin python3

"""
ENPM661 Spring 2022: Planning for Autonomous Robots
Project 2: Dijkstra

Author(s):
Tanuj Thakkar (tanuj@umd.edu)
M. Engg Robotics
University of Maryland, College Park
"""

# Importing Modules
import sys
import os
import numpy as np
import argparse
import time
import math
from queue import PriorityQueue
import matplotlib.pyplot as plt

from Map import *


class Node():
    '''
        Class to represent nodes in Breadth First Search

        Attributes
        state: state of the node
        index: index of the node
        parent_index: index of parent node
        actions: Possible actions to generate child nodes
    '''

    def __init__(self, state: tuple, cost: float, index: int, parent_index: int) -> None:
        self.state = state
        self.cost = cost
        self.index = index
        self.parent_index = parent_index

class Dijkstra:

    def __init__(self, start_state: tuple, goal_state: tuple, occupancy_grid:  np.array, visualize: bool) -> None:
        self.start_state = start_state
        self.goal_state = goal_state
        self.occupancy_grid = occupancy_grid
        self.actions = np.array([[0, 1, 1],
                                 [0, -1, 1],
                                 [-1, 0, 1],
                                 [1, 0, 1],
                                 [-1, 1, np.sqrt(2)],
                                 [-1, -1, np.sqrt(2)],
                                 [1, 1, np.sqrt(2)],
                                 [1, -1, np.sqrt(2)]])
        self.current_index = 0

        self.start_node = Node(self.start_state, 0, self.current_index, None)
        self.goal_node = Node(self.goal_state, 0, -1, None)
        self.open_list = dict()
        self.closed_list = dict()
        self.final_node = None
        self.path = None
        self.visualize = visualize

        print("\nInitialized Dijkstra...\n")
        print("Initial State: \n", self.start_node.state)
        print("Goal State: \n", self.goal_node.state)


    def in_collision(self, pos: np.array, clearance: int) -> bool:
        X, Y = np.ogrid[int(pos[0]) - clearance:int(pos[0]) + clearance, int(pos[1]) - clearance:int(pos[1]) + clearance]
        if(pos[0] < 0 and pos[0] >= self.occupancy_grid.shape[0] and pos[1] < 0 and pos[1] >= self.occupancy_grid.shape[1]):
            # print("Node out of bounds!")
            return True
        elif(not self.occupancy_grid[int(pos[0]),int(pos[1])]):
            # print("Node in collision!")
            return True
        elif(len(np.where(self.occupancy_grid[X, Y] == 0)[0])):
            return True
        else:
            return False

    def to_tuple(self, state: np.array) -> tuple:
        return tuple(state)

    def search(self) -> bool:

        print("\nStarting search...\n")

        pq = PriorityQueue()

        pq.put((self.start_node.cost, self.start_node.state))
        self.open_list[self.start_node.state] = (self.start_node.index, self.start_node)

        if(self.visualize):
            occupancy_grid = np.copy(self.occupancy_grid)
            # plt.plot(np.where(self.occupancy_grid==-1)[0], np.where(self.occupancy_grid==-1)[1], 'ks')
            # plt.plot(np.where(self.occupancy_grid==1)[0], np.where(self.occupancy_grid==1)[1], '.')
            # plt.plot(np.where(self.occupancy_grid>1)[0], np.where(self.occupancy_grid>1)[1], '.k')
            # plt.plot(self.start_state[0], self.start_state[1], 'og')
            # plt.plot(self.goal_state[0], self.goal_state[1], 'xb')
            cv2.imshow("Dijkstra", np.flip(np.uint8(occupancy_grid).transpose(), axis=0))
            cv2.waitKey(0)

        tick = time.time()
        while(not pq.empty()):

            current_node = self.open_list[pq.get()[1]][1]
            self.closed_list[current_node.state] = (current_node.index, current_node)
            del self.open_list[current_node.state]

            if(self.visualize):
                occupancy_grid[int(current_node.state[0]), int(current_node.state[1])] = 0
                cv2.imshow("Dijkstra", np.flip(np.uint8(occupancy_grid).transpose(), axis=0))
                cv2.waitKey(1)

            if(current_node.state == self.goal_node.state):
                print("GOAL REACHED!")
                toc = time.time()
                print("Took %.03f seconds to search the path"%((toc-tick)))
                self.final_node = current_node
                return True

            actions = self.actions
            for action in range(len(actions)):
                new_state = self.to_tuple(current_node.state + actions[action][:2])
                new_index = self.current_index + 1
                new_cost = current_node.cost + actions[action][2]
                self.current_index = new_index
                if(not self.in_collision(new_state, 4)):
                    new_node = Node(new_state, new_cost, new_index, current_node.index)

                    if(new_state in self.closed_list):
                        self.current_index -= 1
                        continue

                    if(new_state not in self.open_list):
                        self.open_list[new_state] = (new_node.index, new_node)
                        pq.put((new_node.cost, new_node.state))
                    else:
                        if(self.open_list[new_state][1].cost > new_node.cost):
                            self.open_list[new_state] = (new_node.index, new_node)
                        else:
                            self.current_index -= 1
                else:
                    self.current_index -= 1
                    # print("NODE IN COLLISION!")
                    pass

        plt.show()
        print("SOLUTION DOES NOT EXIST!")
        return False

    def backtrack_path(self) -> np.array:

        current_node = self.final_node
        self.path = list()
        closed_list = dict(self.closed_list.values())

        print("BACKTRACKING PATH...")

        while(current_node.index != 0):
            self.path.append(current_node.state)
            current_node = closed_list[current_node.parent_index]

        self.path.append(self.start_node.state)
        self.path.reverse()
        self.path = np.array(self.path)

        plt.plot(self.path[:,0], self.path[:,1], '-r')
        plt.plot(self.start_state[0], self.start_state[1], 'og')
        plt.plot(self.goal_state[0], self.goal_state[1], 'xb')
        plt.plot(np.where(self.occupancy_grid==-1)[0], np.where(self.occupancy_grid==-1)[1], 'ks', markersize=0.5)
        plt.plot(np.where(self.occupancy_grid==1)[0], np.where(self.occupancy_grid==1)[1], '.', markersize=0.5)
        plt.plot(np.where(self.occupancy_grid>1)[0], np.where(self.occupancy_grid>1)[1], '.k', markersize=0.5)
        plt.show()

        print("BACKTRACKING PATH COMPLETE!")
        return self.path

# def random_search() -> tuple, tuple:
#     np.random.randint()


def main():

    Parser = argparse.ArgumentParser()
    Parser.add_argument('--StartState', type=str, default="[10, 10]", help='Start state of the robot')
    Parser.add_argument('--GoalState', type=str, default="[390, 215]", help='Goal state of the robot')
    Parser.add_argument('--Random', action='store_true', help='Toggle randomized start and goal states')
    Parser.add_argument('--Visualize', action='store_true', help='Toggle search visualization')

    Args = Parser.parse_args()
    StartState = tuple(map(int, Args.StartState.replace('[', ' ').replace(']', ' ').replace(',', ' ').split()))
    GoalState = tuple(map(int, Args.GoalState.replace('[', ' ').replace(']', ' ').replace(',', ' ').split()))
    Random = Args.Random
    Visualize = Args.Visualize

    m = Map(400,250)
    m.generate_map()

    D = Dijkstra(StartState, GoalState, m.occupancy_grid, Visualize)
    if(D.search()):
        D.backtrack_path()

if __name__ == '__main__':
    main()
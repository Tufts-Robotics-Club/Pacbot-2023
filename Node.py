# NAME: Node.py
# PURPOSE: search space node definition (for use in A*)
# AUTHORS: Rob Pitkin, Emma Bethel, Ryan McFarlane

from variables import *
import numpy as np


def manhattanDist(currPos, goalPos) -> float:
    return abs(currPos[0] - goalPos[0]) + abs(currPos[1] - goalPos[1])


# Class definition of a node in the search space
class Node:
    def __init__(self, grid, pos, goalPos, cost, prev, ghostPos, avoid=False):
        self.grid = grid  # current state of the pacman grid
        self.pos = pos  # grid coordinate represented by node
        self.goalPos = goalPos  # desired coordinates of pacman
        self.cost = cost  # total cost of getting to node
        self.neighbors = []  # possible nodes 1 step further in path
        self.prev = prev  # previous node in path
        self.ghostPos = ghostPos  # list of positions of non-frightened ghosts
        self.avoid = avoid  # depracated; conrols whether pacman should be 
                            #   moving toward or away from goal

    # Overloading the greater than operator so that node comparisons can be made
    # Compares based on the cost of the state
    def __gt__(self, other):
        if self.cost > other.cost:
            return True
        else:
            return False

    # Generating neighbors for a given state.
    def generateNeighbors(self):
        self.neighbors = []
        x = self.pos[0]
        y = self.pos[1]
        newCost = self.cost

        if self.ghostPos:
            ghostDists = []
            for ghost in self.ghostPos:
                ghostDists.append(manhattanDist(self.pos, ghost))
            closestGhost = self.ghostPos[np.argmin(ghostDists)]

        neighborPos = [(x, y + 1), (x, y - 1), (x + 1, y), (x - 1, y)]
        for nx, ny in neighborPos:
            if self.grid[nx][ny] not in [I, n]:
                if self.grid[nx][ny] in self.ghostPos: #Fix this
                    newCost += 9999 # * (0 if self.avoid else 1)
                else:
                    newCost += (1 + manhattanDist((nx, ny), self.goalPos)) * (-1 if self.avoid else 1)
                    if self.ghostPos:
                        newCost += (75.0 / (0.1 + manhattanDist((nx, ny), closestGhost)))

                node = Node(self.grid, (nx, ny), self.goalPos, newCost, self, self.ghostPos, self.avoid)
                self.neighbors.append(node)

    # Method to print neighbors of a node. Used for debugging
    def printNeighbors(self):
        for neighbor in range(len(self.neighbors)):
            print(self.neighbors[neighbor].pos)

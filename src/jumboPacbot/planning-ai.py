from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
import csv
import os
from queue import PriorityQueue

ADDRESS = os.environ.get("BIND_ADDRESS", "localhost")    # address of game engine server
PORT = os.environ.get("BIND_PORT", 11295)               # port game engine server is listening on

SPEED = 1.0

@dataclass
class Planner:
	startLocation = Tuple[int,int]
	pacbotLocation: Tuple[int, int]
	ghostLocations: List[Tuple[int, int]]
	ghostsScared: bool = False

	WALL_COST : int =  100
	GHOST_COST: int =  1000
	COIN_COST : int = -10
	JNCTN_COST: int = -5
	BLANK_COST: int = -1

	COSTS = {
		'#': WALL_COST,
		'G': GHOST_COST,
		'.': COIN_COST,
		' ': BLANK_COST,
		'J': JNCTN_COST
	}
	
	def __repr__(self) -> str:
		return f"Planner(startLocation={self.startLocation}, pacbotLocation={self.pacbotLocation}, ghostLocations={self.ghostLocations}, ghostsScared={self.ghostsScared})"
	
	def __str__(self) -> str:
		return f"{self.pacbotLocation}, {self.ghostLocations}, {self.ghostsScared}"

	def manhattan_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
		return abs(a[0] - b[0]) + abs(a[1] - b[1])
	def is_valid(self, location: Tuple[int,int]) -> bool:
		row, col = location
		return row >=0 and row<len(self.board) and col >=0 and col<len(self.board[0]) and self.board[row][col] != '#' and self.board[row][col] != 'X'
	
	def init_board(self):
		with open('board.csv', newline='') as csvfile:
			self.board = np.array(list(csv.reader(csvfile)))

	def plan(self):

		if self.is_junction(self.pacbotLocation):
			# Pacbot is at a junction
			
			if self.ghostsScared:
				# Chase the ghosts
				goal = ... # Find the closest ghost
				plan = ...
			else:
				# Go after coins, avoid ghosts
				goal = ... # Find the closest coin???
		else:
			# Pacbot is not at a junction

			# Check if there are any ghosts on the path to the next junction, if the ghosts are not scared

			current_path = ...
			ghosts_on_path = ... # Check if there are any ghosts on the path to the next junction

			if ghosts_on_path and not self.ghostsScared:
				# There are ghosts on the path to the next junction
				
				... # Backtrack to the previous junction
				
				plan = ...

		return plan
	
	def plan_path(self, goal):
		# Plan the path to the next goal

		"""
		1. Get all possible paths to the goal
		2. Find subpaths that rejoin at a common junction
		3. Calculate heuristic for each subpath and choose the best one
		   - total length of subpath
		   - number of junctions
		   - number of coins
		   - proximity to ghosts (OR) distance of ghosts to the endpoint of the subpath
		"""

		directions = [(0,1), (1,0), (0,-1), (-1,0)]
		path_depth = 4
		
		paths = []
		for dir in directions:
			path_indices = [(
				max(0, min(self.pacbotLocation[0] + dir[0]*i, self.board.shape[0])), 
				max(0, min(self.pacbotLocation[1] + dir[1]*i, self.board.shape[1]))
			) for i in range(path_depth+1)]

			path_cost = 0
			path_heuristic = 0
			for i, loc in enumerate(path_indices):
				path_cost += self.COSTS[self.board[loc]]
				path_cost += self.COSTS['J'] * self.is_junction(loc)

				if self.board[loc] != '#':
					path_heuristic += (path_depth - i) * self.manhattan_distance(loc, goal)

				for ghostLoc in self.ghostLocations:
					path_cost += self.COSTS['G'] * (i == ghostLoc)

			paths.append((dir, path_cost, path_heuristic))
		
		print(paths)

		return
	
	def is_junction(self, location: Tuple[int, int]) -> bool:
		row, col = location
		row_prev, col_prev = row-1, col-1
		row_next, col_next = row+1, col+1

		neighbors = [
			(row_prev, col),
			(row, col_next),
			(row_next, col),
			(row, col_prev)
		]

		return np.sum([self.board[neighbor] != '#' for neighbor in neighbors]) > 2

	def run(self) -> List[Tuple[int, int]]:
		frontier = PriorityQueue()
		frontier.put(self.startLocation, 0)


# Testing the Planner class
	
# Create an instance of the Planner class
planner = Planner((23,13), (23,13), [(1,1), (2,2)])
print(planner)
planner.init_board()
planner.plan_path((3, 3))

# Call the plan() method to test it
planner.plan()  # Output: Running algorithm for scared ghost...

# Change the value of the ghostsScared attribute and call the plan() method again
planner.ghostsScared = False
planner.plan()  # Output: Running algorithm for normal ghost...

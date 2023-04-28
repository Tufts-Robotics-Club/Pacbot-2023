from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
import csv
import os
from queue import PriorityQueue
from messages import MsgType, message_buffers, PacmanDirection, LightState

ADDRESS = os.environ.get("BIND_ADDRESS", "localhost")    # address of game engine server
PORT = os.environ.get("BIND_PORT", 11295)               # port game engine server is listening on

SPEED = 1.0

FREQUENCY = SPEED * game_frequency

BASE_TICK_COUNTER = 1

# times are measured in ticks (calls of the planning function), not seconds.
#   could change this later (& just decrement by 1/FREQUENCY instead of 1
#   during each tick)
FRUIT_TIME = 10 * FREQUENCY
FRIGHTENED_TIME = 20 * FREQUENCY

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

	goal = (1,1)
	last_location = None
	direction = (1,0)
	fruit_timer = 0

	COSTS = {
		'#': WALL_COST,
		'G': GHOST_COST,
		'.': COIN_COST,
		' ': BLANK_COST,
		'J': JNCTN_COST
	}

	char_to_direction = {
		'w': PacmanDirection.W,
		'a': PacmanDirection.A,
		's': PacmanDirection.S,
		'd': PacmanDirection.D,
		'q': PacmanDirection.STOP
	}
	
	def __repr__(self) -> str:
		return f"Planner(startLocation={self.startLocation}, pacbotLocation={self.pacbotLocation}, ghostLocations={self.ghostLocations}, ghostsScared={self.ghostsScared})"
	
	def __str__(self) -> str:
		return f"{self.pacbotLocation}, {self.ghostLocations}, {self.ghostsScared}"
	
	def msg_received(self, msg, msg_type):
	    # This gets called whenever any message is received
	    # This module only sends data, so we ignore incoming messages
		if msg_type == MsgType.LIGHT_STATE:
			self.state = msg
			old_pos = self.pacbot_pos
			self.pacbot_pos = (msg.pacman.x, msg.pacman.y)
			if old_pos != self.pacbot_pos:
				self.needs_to_plan = True
				self.tick_counter = BASE_TICK_COUNTER
			print("message position:", msg.pacman.x, msg.pacman.y)
			if self.state.lives != self.lives:
				self.lives = self.state.lives
				self.pacbot_pos = [pacbot_starting_pos[0], pacbot_starting_pos[1]]

	def manhattan_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
		return abs(a[0] - b[0]) + abs(a[1] - b[1])
	
	def is_valid(self, location: Tuple[int,int]) -> bool:
		row, col = location
		return row >=0 and row<len(self.board) and col >=0 and col<len(self.board[0]) and self.board[row][col] != '#' and self.board[row][col] != 'X'
	
	def init_board(self):
		with open('board.csv') as csvfile:
			self.board = np.array(list(csv.reader(csvfile)))

	def find_closest_coin(self):
		min_distance = float('inf')
		closest_coin = None
		for coin in self.coins:
			distance = manhattan_distance(self.position, coin.position)
			if distance < min_distance:
				min_distance = distance
				closest_coin = coin
		return closest_coin


	def plan(self):

		if self.last_location is None:
			self.last_location = (self.pacbotLocation[0] - self.direction[0], self.pacbotLocation[1] - self.direction[1])

		self.direction = (self.pacbotLocation[0] - self.last_location[0], self.pacbotLocation[1] - self.last_location[1])

		if self.is_junction(self.pacbotLocation):
			# Pacbot is at a junction
			print("Pacbot is at a junction")
			
			if self.ghostsScared:
				print("Ghosts are scared")

				# Chase the ghosts
				
				# Find the closest ghost
				self.goal = self.ghostLocations[np.argmin([self.manhattan_distance(self.pacbotLocation, ghostLoc) for ghostLoc in self.ghostLocations])]
			else:
				print("Ghosts are scary")

				# Go after coins, avoid ghosts
				if  self.manhattan_distance((13,13), self.pacbotLocation) < self.fruit_timer:
					self.goal = (13,13)
				else:
					self.goal = find_closest_coin(self.pacbotLocation, coins) # Find the closest coin???
		else:
			# Pacbot is not at a junction
			print("Pacbot is not at a junction")

			# Check if there are any ghosts on the path to the next junction, if the ghosts are not scared

			current_path = []
			i = 1
			while self.board[self.pacbotLocation[0] + self.direction[0]*i][self.pacbotLocation[1] + self.direction[1]*i] != '#':
				current_path.append((self.pacbotLocation[0] + self.direction[0]*i, self.pacbotLocation[1] + self.direction[1]*i))
				i += 1

			# Check if there are any ghosts on the path to the next junction
			ghosts_on_path = any([ghostLoc in current_path for ghostLoc in self.ghostLocations])

			if ghosts_on_path and not self.ghostsScared:
				# There are ghosts on the path to the next junction
				print("There are ghosts on the path, turning around")
				
				# Backtrack to the previous junction
				i = 1
				while self.board[self.pacbotLocation[0] - self.direction[0]*i][self.pacbotLocation[1] - self.direction[1]*i] != '#':
					self.goal = (self.pacbotLocation[0] + self.direction[0]*i, self.pacbotLocation[1] + self.direction[1]*i)
					i += 1

		plan = self.plan_path(self.goal)
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
		
		optimal_dir = min(paths, key=lambda x: x[1] + x[2])

		return optimal_dir[0]
	
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
planner.init_board()

# Call the plan() method to test it
planner.plan()  # Output: Running algorithm for scared ghost...

# Change the value of the ghostsScared attribute and call the plan() method again
planner.ghostsScared = False
planner.plan()  # Output: Running algorithm for normal ghost...

for i in range(1000):
	res = planner.plan()
	print(f"At {planner.pacbotLocation}, moving in direction {res}")
	planner.pacbotLocation = (planner.pacbotLocation[0] + res[0], planner.pacbotLocation[1] + res[1])

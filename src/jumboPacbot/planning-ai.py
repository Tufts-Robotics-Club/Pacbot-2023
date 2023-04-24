from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
import csv

@dataclass
class Planner:
	pacbotLocation: Tuple[int, int]
	ghostLocations: List[Tuple[int, int]]
	ghostsScared: bool = False
	
	def __repr__(self) -> str:
		return f"Planner(pacbotLocation={self.pacbotLocation}, ghostLocations={self.ghostLocations}, ghostsScared={self.ghostsScared})"
	
	def __str__(self) -> str:
		return f"{self.pacbotLocation}, {self.ghostLocations}, {self.ghostsScared}"
	
	def initialize_coins(self):
		with open('board.csv', newline='') as csvfile:
			data = np.array(list(csv.reader(csvfile)))
		
	
	def plan(self):
		self.initialize_coins()
		
		if self.pacbotLocation in self.junctions:
			# Pacbot is at a junction
			
			if self.ghostsScared:
				# Chase the ghosts
				plan = self.plan_ghosts_scared()
			else:
				# Go after coins, avoid ghosts
				plan = self.plan_ghosts_scary()
		else:
			# Pacbot is not at a junction

			# Check if there are any ghosts on the path to the next junction, if the ghosts are not scared
			...

		return plan

	def plan_ghosts_scared(self):
		# Chase the ghosts
		print("Chasing ghosts...")
		
		goal = ...
		
		plan = self.plan_path(goal)
		return plan

	def plan_ghosts_scary(self):
		# Go after coins, avoid ghosts
		print("Collecting coins...")

		goal = ...

		plan = self.plan_path(goal)
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
		
		path_heuristic = ...

		return


# Testing the Planner class
	
# Create an instance of the Planner class
planner = Planner((0, 0), [(1, 1), (2, 2)], ghostsScared=True)
print(planner)

# Call the plan() method to test it
planner.plan()  # Output: Running algorithm for scared ghost...

# Change the value of the ghostsScared attribute and call the plan() method again
planner.ghostsScared = False
planner.plan()  # Output: Running algorithm for normal ghost...
#!/usr/bin/env python3

# NAME: aiModule.py
# PURPOSE: module for analyzing current game state and generating and sending 
#          pacman movement commands accordingly
# AUTHORS: Emma Bethel, Rob Pitkin, Ryan McFarlane

import os
import robomodules as rm
from messages import MsgType, message_buffers, PacmanDirection, LightState
from Node import manhattanDist
from variables import *
from grid import *
import numpy as np

###### AI INPUT STUFF ########
import smarterInput
##############################

ADDRESS = os.environ.get("BIND_ADDRESS", "localhost")    # address of game engine server
PORT = os.environ.get("BIND_PORT", 11295)               # port game engine server is listening on

SPEED = 1.0
# SPEED = 5.0 # originally 1.0
FREQUENCY = SPEED * game_frequency

FRUIT_POS = (13, 13)
GHOST_AVOID_RANGE = 6.0

# TODO: should be an actual experimentally- determined robot speed estimate
SQUARES_PER_SEC = SPEED * FREQUENCY

BASE_TICK_COUNTER = 1

# times are measured in ticks (calls of the planning function), not seconds.
#   could change this later (& just decrement by 1/FREQUENCY instead of 1
#   during each tick)
FRUIT_TIME = 10 * FREQUENCY
FRIGHTENED_TIME = 20 * FREQUENCY

GRID_WIDTH = len(grid)
GRID_HEIGHT = len(grid[0])

char_to_direction = {
    'w': PacmanDirection.W,
    'a': PacmanDirection.A,
    's': PacmanDirection.S,
    'd': PacmanDirection.D,
    'q': PacmanDirection.STOP
}

class AIModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)

        self.pacbot_pos = [pacbot_starting_pos[0], pacbot_starting_pos[1]]
        self.state = LightState()
        self.state.mode = LightState.GameMode.PAUSED
        self.lives = starting_lives
        self.avoid = False
        self.grid = grid
        self.fruit_timer = 0
        self.ghost_timer = 0
        self.prev_fruit_val = False
        self.needs_to_plan = True
        self.tick_counter = BASE_TICK_COUNTER
        self.ticks_since_last_plan = 0


    def msg_received(self, msg, msg_type):
        # This gets called whenever any message is received
        # This module only sends data, so we ignore incoming messages
        if msg_type == MsgType.LIGHT_STATE:
            self.state = msg
            old_pos = self.pacbot_pos
            self.pacbot_pos = (msg.pacman.x, msg.pacman.y)
            if old_pos != self.pacbot_pos or self.ticks_since_last_plan >= 10:
                self.needs_to_plan = True
                # self.tick_counter = BASE_TICK_COUNTER
            print("message position:", msg.pacman.x, msg.pacman.y)
            if self.state.lives != self.lives:
                self.lives = self.state.lives
                self.pacbot_pos = [pacbot_starting_pos[0], pacbot_starting_pos[1]]

    def tick(self):
        msg = PacmanDirection()

        if self.tick_counter > 0:
            self.tick_counter -= 1
            if self.tick_counter > 0:
                return

        if self.state.mode != LightState.GameMode.PAUSED and self.needs_to_plan:

            # decrement timers
            if self.ghost_timer > 0:
                self.ghost_timer -= 1
            if self.fruit_timer > 0:
                self.fruit_timer -= 1

            if self.grid[self.pacbot_pos[0]][self.pacbot_pos[1]] != e:
                # if just ate a power pellet, start frightened timer
                if self.grid[self.pacbot_pos[0]][self.pacbot_pos[1]] == O:
                    self.ghost_timer = FRIGHTENED_TIME
                # mark current spot as eaten
                self.grid[self.pacbot_pos[0]][self.pacbot_pos[1]] = e
            
                        # if fruit just appeared, start fruit timer
            if self.state.cherry and not self.prev_fruit_val and self.fruit_timer == 0:
                print('starting fruit timer')
                self.fruit_timer = FRUIT_TIME
            self.prev_fruit_val = self.state.cherry
        
            ghosts = [self.state.red_ghost, self.state.pink_ghost,
                      self.state.blue_ghost, self.state.orange_ghost]

            non_scared_ghosts = [ghost for ghost in ghosts if ghost.state != LightState.GhostState.FRIGHTENED]

            goal_pos = self._pick_goal(ghosts, self.grid)

            if goal_pos is not None:
                out_char = smarterInput.whichWayAStar(self.grid, self.pacbot_pos, 
                                                        goal_pos, non_scared_ghosts, False)
                self.needs_to_plan = False
                self.ticks_since_last_plan = 0
            else:
                out_char = 'q'
            print('OUT CHAR: ' + out_char)
            msg.direction = char_to_direction[out_char]
            self.write(msg.SerializeToString(), MsgType.PACMAN_DIRECTION)
        elif self.needs_to_plan:
            msg.direction = PacmanDirection.STOP

            self.write(msg.SerializeToString(), MsgType.PACMAN_DIRECTION)
        else:
            self.ticks_since_last_plan += 1

    # PURPOSE: picks the ideal goal position for pacman to move toward based on 
    #          current game state
    # PARAMETERS: ghosts - list of all ghosts (as GhostAgent objects) currently 
    #                      in the game (both frigtened and non-frightened)
    #             grid - the current state of the grid, with all eaten pellets 
    #                     marked accordingly
    # RETURNS: a chosen goal coordinate (x,y) or NoneType if the pacman should 
    #          halt
    def _pick_goal(self, ghosts, grid):
        frightened_ghosts = [ghost for ghost in ghosts if ghost.state == LightState.GhostState.FRIGHTENED]

        # ghost eating mode
        if len(frightened_ghosts) != 0:
            frightenedPos = [(ghost.x, ghost.y) for ghost in frightened_ghosts if grid[ghost.x][ghost.y] != n]
            closestGhost = []
            for p in frightenedPos:
                closestGhost.append(abs(self.pacbot_pos[0] - p[0]) + abs(self.pacbot_pos[1] - p[1]))
            goalPos = frightenedPos[np.argmin(closestGhost)]

            if manhattanDist(self.pacbot_pos, goalPos) <= SQUARES_PER_SEC * (self.ghost_timer / FREQUENCY):
                print(f'going for ghost at {goalPos}')
                return goalPos
            print('GHOSTS TOO FAR', self.ghost_timer, goalPos)
        
        # fruit eating mode
        if self.state.cherry and manhattanDist(self.pacbot_pos, FRUIT_POS) <= SQUARES_PER_SEC * (self.fruit_timer / FREQUENCY):
            print('going for fruit!!!!')
            return FRUIT_POS
        
        # power pellet eating mode
        powerPos = [(1, 7), (1, 27), (26, 7), (26, 27)]
        closestPower = []
        for index in range(len(powerPos) -1, -1, -1):
            pos = powerPos[index]
            if grid[pos[0]][pos[1]] != e:
                closestPower.append(abs(self.pacbot_pos[0] - pos[0]) + abs(self.pacbot_pos[1] - pos[1]))
            else:
                powerPos.remove(pos)
        closestPower.reverse()
        if len(closestPower) != 0:
            goalPos = powerPos[np.argmin(closestPower)]
            print(f'going for power pellet at {goalPos}')
            # HERE IS THE LINE FOR STALLING
            if self._should_eat_pellet(goalPos, ghosts):
                return goalPos
            return None
        
        # regular pellet eating mode
        ghostDists = []
        # only avoiding ghosts that aren't in the ghost zone (otherwise path 
        #   planning returns a NoneType)
        dangerous_ghosts = [ghost for ghost in ghosts if grid[ghost.x][ghost.y] != n]
        for ghost in dangerous_ghosts:
            ghostDists.append(manhattanDist(self.pacbot_pos, (ghost.x, ghost.y)))
        if not self.avoid and min(ghostDists) < GHOST_AVOID_RANGE:
            self.avoid = True
        if self.avoid and min(ghostDists) > GHOST_AVOID_RANGE + 1:
            self.avoid = False

        if not self.avoid:
            goalPos = smarterInput.findClosestBFS(grid, *self.pacbot_pos)[:2]
            print(f'going for regular pellet at {goalPos}')
            return goalPos
        else:
            closestGhost = ghosts[np.argmin(ghostDists)]
            print(f'avoiding ghost at {(closestGhost.x, closestGhost.y)}')
            goalPos = self._get_mirror_pos((closestGhost.x, closestGhost.y))
            print(f'mirror pos {goalPos}')
            return goalPos

    # PURPOSE: determines whether pacman should continue moving toward a power 
    #          pellet or stall near it to wait for ghosts
    # PARAMETERS: pellet_pos - the coordinates of the pellet
    #             ghosts - list of ghost objects that will become edible after 
    #                      power pellet is eaten
    # RETURNS: True to continue moving, False otherwise
    def _should_eat_pellet(self, pellet_pos, ghosts):
        if manhattanDist(self.pacbot_pos, pellet_pos) > 1:
            print('FAR FROM PELLET')
            return True

        counter = 0
        for ghost in ghosts:
            dist = manhattanDist(self.pacbot_pos, (ghost.x, ghost.y))
            if dist <= 3:
                print('GHOST TOO CLOSE')
                return True
            if dist <= SQUARES_PER_SEC * (FRIGHTENED_TIME / FREQUENCY) / 4:
                counter += 1
            if counter >= 2:
                print('GHOSTS CLOSE ENOUGH')
                return True

        return False

    # PURPOSE: calculates coordinate of furthest corner from a given grid 
    #          position
    # PARAMETERS: pos - the grid position as an (x,y) tuple
    # RETURNS: the grid coordinates of the furthest corner (x,y)
    def _get_mirror_pos(self, pos):
        if pos[0] < GRID_WIDTH / 2:
            x = GRID_WIDTH - 2
        else:
            x = 1
        if pos[1] < GRID_HEIGHT / 2:
            y = GRID_HEIGHT - 2
        else:
            y = 1
        return (x, y)


def main():
    module = AIModule(ADDRESS, PORT)
    module.run()

if __name__ == "__main__":
    main()

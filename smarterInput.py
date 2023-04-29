# NAME: smarterInput.py
# PURPOSE: A* and BFS implementations for picking best direction for bot to go 
#          in given current game state
# AUTHORS: Ryan McFarlane, Rob Pitkin, Emma Bethel

from variables import *
import Node
import HQ


# Generates the path taken from the start node to the parameter node (typically
# the goal state)
def generateSolution(node, start):
    solution = []
    while node.pos != start.pos:
        solution.append(node.pos)
        node = node.prev
    solution.append(start.pos)
    solution.reverse()

    return solution


# take in the grid and the pacman position
# return which direction to go
def whichWay(grid, pacpos):
    x = pacpos[0]
    y = pacpos[1]
    if grid[x][y] in [e]:
        print("already eaten!")
    grid[x][y] = e
    if grid[x - 1][y] in [o, O]:
        return 'a', grid
    elif grid[x][y + 1] in [o, O]:
        return 'w', grid
    elif grid[x + 1][y] in [o, O]:
        return 'd', grid
    elif grid[x][y - 1] in [o, O]:
        return 's', grid
    else:  # not immediately next to a dot, find the closest one
        print("\nsearching for a dot!!\n")
        # look through every dot in the grid, return the closest
        goalX, goalY = findClosest(grid, x, y)
        print("Closest dot is at: " + str(goalX) + ", " + str(goalY))
        # go to the dot at goalX, goalY

        if goalX > x:  # we have to go right
            if grid[x + 1][y] not in [I, n]:  # not blocked
                return 'd', grid
            if goalY > y:  # we have to go up
                if grid[x][y + 1] not in [I, n]:  # not blocked
                    return 'w', grid
                # can't go up or right but need to
                if grid[x][y - 1] not in [I, n]:  # not blocked
                    return 's', grid
                return 'a'

        if goalX < x:  # we have to go left
            if grid[x - 1][y] not in [I, n]:  # not blocked
                return 'a', grid
            if goalY > y:  # we have to go up
                if grid[x][y + 1] not in [I, n]:  # not blocked
                    return 'w', grid
                if grid[x][y - 1] not in [I, n]:  # not blocked
                    return 's', grid
                return 'd'

        if goalY > y:  # we have to go up
            if grid[x][y + 1] not in [I, n]:  # not blocked
                return 'w', grid
            if goalY > y:  # we have to go up
                if grid[x + 1][y] not in [I, n]:  # not blocked
                    return 'd', grid
                if grid[x - 1][y] not in [I, n]:  # not blocked
                    return 'a', grid
                return 's'

        if goalY < y:  # we have to go down
            if grid[x][y - 1] not in [I, n]:  # not blocked
                return 's', grid
            if goalY > y:  # we have to go up
                if grid[x + 1][y] not in [I, n]:  # not blocked
                    return 'd', grid
                if grid[x - 1][y] not in [I, n]:  # not blocked
                    return 'a', grid
                return 'w'

        return 'a', grid


# PURPOSE: uses BFS to determine the best direction for pacman to move in order 
#          to reach nearest a power pellet
# PARAMETERS: grid - the current state of the pacman grid, with already-eaten 
#                    pellets marked accordingly
#             pacpos - current coordinates of pacman
# RETURNS: character representing desired direction (w, a, s, or d)
# NOTE: does not handle ghost avoidance
def whichWayBFS(grid, pacpos):
    x = pacpos[0]
    y = pacpos[1]

    if grid[x][y] in [e]:
        print("already eaten!")

    if grid[x - 1][y] in [o, O]:
        return 'a'
    elif grid[x][y + 1] in [o, O]:
        return 'w'
    elif grid[x + 1][y] in [o, O]:
        return 'd'
    elif grid[x][y - 1] in [o, O]:
        return 's'
    else:  # not immediately next to a dot, find the closest one
        print("\nsearching for a dot!!\n")
        # look through every dot in the grid, return the closest
        # returns x, y, path
        goalX, goalY, path = findClosestBFS(grid, x, y)
        print("Closest dot is at: " + str(goalX) + ", " + str(goalY))
        # go to the dot at goalX, goalY
        print("Next move is to: " + str(path[1]))
        print("Next move is to: " + str(path))
        if path[1][0] > x:  # we have to go right
            return 'd',
        elif path[1][0] < x:  # we have to go left
            return 'a'
        elif path[1][1] > y:  # we have to go up
            return 'w'
        elif path[1][1] < y:
            return 's'
        else:
            print("\n****\n****\nLINE\n\n\n")
            print(path[0])
            print(path[1])
            print("\n****\n****\nLINE\n\n\n")
            return 's'


# PURPOSE: uses an A* search to determine the best direction for pacman to 
#          move in in order to reach a given goal position
# PARAMETERS: grid - the current state of the pacman grid, with already-eaten 
#                    pellets marked accordingly
#             pacpos - current coordinates of pacman
#             goalPos - the goal coordinates (x,y)
#             ghosts - list of all non-frightened ghosts (as GhostAgent 
#                       objects) currently in the game
#             avoid - depracated; was used in an earlier version of the regular 
#                     pellet eating/ghost avoidance code
# RETURNS: character representing desired direction (w, a, s, or d)
def whichWayAStar(grid, pacpos, goalPos, ghosts, avoid=False):
    x = pacpos[0]
    y = pacpos[1]
    ghostPos = [(g.x,g.y) for g in ghosts]
    print("My position is", x, y)

    path = findClosestAStar(grid, x, y, goalPos, ghostPos, avoid)
    # go to the dot at goalX, goalY
    if path is None:  # in theory, this should never happen
        print(f'No path found! Pacpos {pacpos}, goalpos {goalPos}, ghosts {ghosts}')

    print("path",  path)
    print("Next move is to: " + str(path[1]))
    if path[1] in ghostPos:
        # Our next move is directly into a ghost
        valid_directions = {'w','a','s','d'}
        if path[1][0] > x:  # we have to go right
            valid_directions.remove('d')
        elif path[1][0] < x:  # we have to go left
            valid_directions.remove('a')
        elif path[1][1] > y:  # we have to go up
            valid_directions.remove('w')
        elif path[1][1] < y:
            valid_directions.remove('s')
        for d in valid_directions:
            if d == 'w' and grid[x][y+1] not in [I, n]:
                return 'w'
            if d == 'a' and grid[x - 1][y] not in [I, n]:
                return 'a'
            if d == 's' and grid[x][y - 1] not in [I, n]:
                return 's'
            if d == 'd' and grid[x + 1][y] not in [I, n]:
                return 'd'

    if path[1][0] > x:  # we have to go right
        return 'd'
    elif path[1][0] < x:  # we have to go left
        return 'a'
    elif path[1][1] > y:  # we have to go up
        return 'w'
    elif path[1][1] < y:
        return 's'
    else:
        return 's'


def distTo(x, y, newx, newy):
    return ((newx - x) ** 2 + (newy - y) ** 2) ** 1 / 2


def findClosest(grid, x, y):
    closestDot = -1, -1
    closestDist = 1000
    for xs in range(len(grid)):
        for ys in range(len(grid[0])):
            if grid[xs][ys] in [o, O]:
                dist = distTo(x, y, xs, ys)
                if dist < closestDist:
                    closestDist = dist
                    closestDot = xs, ys
    return closestDot


# PURPOSE: uses breadth-first search to find shortest from pacman to nearest 
#          pellet
# PARAMETERS: grid - the current state of the pacman grid, with already-eaten 
#                    pellets marked accordingly
#             x - current x coordinate of pacman
#             y - current y coordinate of pacman
# RETURNS: tuple containing the (x,y) coordinate of that nearest pellet, as 
#          well as the path (as a list of coordinates) to it
def findClosestBFS(grid, x, y):
    childPath = []
    queue = []
    queue.append((x, y, childPath))
    while len(queue) > 0:
        args = queue.pop(0)
        childX = args[0]
        childY = args[1]
        childPath = args[2]
        if (childX, childY) not in childPath:
            childPath.append((childX, childY))
            if grid[childX][childY] in [o, O]:  # found the dot
                return (childX, childY, childPath)
            if grid[childX - 1][childY] not in [I, n]:  # not blocked
                queue.append((childX - 1, childY, childPath.copy()))
            if grid[childX + 1][childY] not in [I, n]:  # not blocked
                queue.append((childX + 1, childY, childPath.copy()))
            if grid[childX][childY - 1] not in [I, n]:  # not blocked
                queue.append((childX, childY - 1, childPath.copy()))
            if grid[childX][childY + 1] not in [I, n]:  # not blocked
                queue.append((childX, childY + 1, childPath.copy()))


# PURPOSE: uses an A* search to find the lowest-cost path from pacman to 
#          given goal position
# PARAMETERS: grid - the current state of the pacman grid, with already-eaten 
#                    pellets marked accordingly
#             x - current x coordinate of pacman
#             y - current y coordinate of pacman
#             goalPos - the goal coordinates (x,y)
#             ghostPos - list of coordinates (x,y) of all non-frightened ghosts 
#                        currently in the game
#             avoid - depracated; was used in an earlier version of the regular 
#                     pellet eating/ghost avoidance code
# RETURNS: the last node in the path (which can be traced back through the 
#          entire path)
def findClosestAStar(grid, x, y, goalPos, ghostPos, avoid=False):
    pq = HQ.HQ()
    start = Node.Node(grid, (x, y), goalPos, 0, None, ghostPos, avoid)
    pq.insert(start)
    visited = set()
    nodeNumber = 0
    while True:
        # Checking if the priority queue is empty. In other words, making sure
        # there are nodes to visit.
        if pq.isEmpty():
            print("Failure")
            break
        node = pq.remove()
        nodeNumber += 1
        # "Marking" (adding to a list) the state as visited
        visited.add(node.pos)
        # Checks if the current state is the goal state
        if node.pos == goalPos:
            print("Done!")
            print("Num states visited:", len(visited))
            return generateSolution(node, start)
        # Generating neighbors of the current state and adding them to the PQ
        # if not visited already
        node.generateNeighbors()

        for i in range(len(node.neighbors)):
            # Checking if the neighbor was visited or already in the PQ
            if node.neighbors[i].pos not in visited and (pq.contains(node.neighbors[i].pos) == -1):
                pq.insert(node.neighbors[i])
            # Updating cost if necessary
            elif pq.contains(node.neighbors[i].pos) > -1:
                index = pq.contains(node.neighbors[i].pos)
                newCost = node.neighbors[i].cost
                currentCost = pq.q[index].cost
                if newCost < currentCost:
                    pq.q[index] = node.neighbors[i]

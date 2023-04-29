# NAME: HQ.py
# PURPOSE: priority queue implementation, for use in A* search
# AUTHORS: Rob Pitkin

import heapq


class HQ:
    def __init__(self):
        self.q = []
        heapq.heapify(self.q)

    # Checks if the PQ is empty
    def isEmpty(self):
        return len(self.q) == 0

    # Inserts a pos tuple into the PQ
    def insert(self, pos):
        heapq.heappush(self.q, pos)

    # Removes a pos tuple from the PQ
    def remove(self):
        if self.isEmpty():
            print("Empty Queue")
            return 0
        nextState = heapq.heappop(self.q)
        return nextState

    # Checks if a pos is in the PQ, returns its index if it is, otherwise
    # returns -1 (false)
    def contains(self, pos):
        for i in range(len(self.q)):
            if self.q[i].pos == pos:
                return i
        return -1

    # Prints the PQ. Used for debugging
    def printQueue(self):
        for i in range(len(self.q)):
            print(self.q[i].pos, self.q[i].cost)

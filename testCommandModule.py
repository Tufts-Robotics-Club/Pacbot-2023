#!/usr/bin/env python3

# NAME: testCommandModule.py
# PURPOSE: module for driving robot via keyboard input. for use in testing.
# AUTHORS: Emma Bethel

import os
import robomodules as rm
from variables import *
from messages import MsgType, message_buffers, PacmanDirection

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 60


# drive robot based on command line input
class TestCommandModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = []
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None
        self.last_command = None

    def msg_received(self, msg, msg_type):
       pass

    def tick(self):
        new_msg = PacmanDirection()
        new_msg.direction = self._get_direction()
        print(f'sending: {new_msg.direction}')
        self.write(new_msg.SerializeToString(), MsgType.PACMAN_DIRECTION)

    # PURPOSE: translates keyboard input (w/a/s/d/q) into pacman direction enum 
    #          value
    # PARAMETERS: N/A
    # RETURNS: the enum value of the requested direction
    def _get_direction(self):
        print('enter robot direction (w/a/s/d/q):', end='')
        command = input().lower()
        if command == 'w':
            return PacmanDirection.W
        elif command == 'a':
            return PacmanDirection.A
        elif command == 's':
            return PacmanDirection.S
        elif command == 'd':
            return PacmanDirection.D
        elif command == 'q':
            return PacmanDirection.STOP
        else:
            print('invalid input. try again.')
            return self._get_direction()


def main():
    module = TestCommandModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()

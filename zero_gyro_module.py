#!/usr/bin/env python3

# NAME: testCommandModule.py
# PURPOSE: module for driving robot via keyboard input. for use in testing.
# AUTHORS: Emma Bethel

import os
import robomodules as rm
from variables import *
from messages import MsgType, message_buffers, GyroYaw

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 60


# drive robot based on command line input
class TestCommandModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = []
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)

    def msg_received(self, msg, msg_type):
       pass

    def tick(self):
       input()
       new_msg = GyroYaw()
       new_msg.yaw = 0
       self.write(new_msg.SerializeToString(), MsgType.GYRO_YAW)
       print("sent zero message")

def main():
    module = TestCommandModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()

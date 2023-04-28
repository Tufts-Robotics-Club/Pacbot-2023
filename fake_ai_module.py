import robomodules as rm
from messages import MsgType, message_buffers, PacmanDirection, GyroYaw

import os

ADDRESS = os.environ.get("LOCAL_ADDRESS", "localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

class FakeAiModule(rm.ProtoModule):

    def __init__(self, addr, port):
        self.FREQUENCY = 100

        self.subscriptions = [MsgType.PACMAN_DIRECTION]
        super().__init__(addr, port, message_buffers, MsgType, self.FREQUENCY, self.subscriptions)
        self.count = 0
        WASD_to_direction = {
            "W": 1,
            "A": 2,
            "S": 3,
            "D": 4
        }
        self.action_queue = [WASD_to_direction[action.lower()] for action in input("Enter actions: ").split(" ")]

    def tick(self):
        self.count += 1
        if self.count % 200 == 0 and len(self.action_queue) > 0:
            print("Sending message", self.action_queue[0])
            msg = PacmanDirection()
            msg.direction = self.action_queue[0]
            self.action_queue = self.action_queue[1:]
            self.write(msg.SerializeToString(), MsgType.PACMAN_DIRECTION)

    def msg_received(self, msg, msg_type):
        pass

def main():
    print("Running fake AI module")
    module = FakeAiModule(ADDRESS, PORT)
    module.run()

if __name__ == "__main__":
    main()
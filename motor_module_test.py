import robomodules as rm
from messages import MsgType, message_buffers, PacmanDirection, GyroYaw
from gpiozero import PhaseEnableMotor, RotaryEncoder
from enum import IntEnum
import math
import os

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

Direction = IntEnum("Direction", ["FORWARD", "LEFT", "BACKWARD", "RIGHT"])

class MotorModule(rm.ProtoModule):
    def __init__(self, addr, port):
        
        # Constants \U+1F929
        self.FREQUENCY = 100

        # How far from the target distance is acceptible before stopping
        self.STOPPING_ERROR = 0.1
        # How much the two wheels can be different before we try to compensate
        self.DIFFERENCE_ERROR = 0.1

        self.TURN_SPEED = 1.0
        self.TURN_DISTANCE = 1.0
        self.CATCHUP_MODIFIER = 1.1
        self.MOVE_SPEED = 1.0
        # 6 inches / (Wheel diameter * pi * 1 in / 25.2 mm)
        self.MOVE_ROTATIONS = 6 / (32 * math.pi / 25.4)

        self.LEFT_MOTOR_PINS = (19, 26)
        self.RIGHT_MOTOR_PINS = (5, 6)
        self.LEFT_ENCODER_PINS = (23, 24)
        self.RIGHT_ENCODER_PINS = (14, 15)


        # Need to set up connections and stuff
        self.subscriptions = [MsgType.PACMAN_DIRECTION, MsgType.GYRO_YAW, MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, self.FREQUENCY, self.subscriptions)

        # Motors - have to change the pins or whatever
        self.left_motor = PhaseEnableMotor(*self.LEFT_MOTOR_PINS)
        self.right_motor = PhaseEnableMotor(*self.RIGHT_MOTOR_PINS)
        # Encoders
        self.left_encoder = RotaryEncoder(*self.LEFT_ENCODER_PINS, max_steps=0)
        self.right_encoder = RotaryEncoder(*self.RIGHT_ENCODER_PINS, max_steps=0)

        self.left_target = self.left_encoder.steps
        self.right_target = self.right_encoder.steps
        self.current_direction = Direction.FORWARD
        self.action_queue = []

        
    # Turns to the the increments * 90 degrees
    def _turn_real(self, increments: int):
        self.left_target += self.TURN_DISTANCE * increments
        self.right_target -= self.TURN_DISTANCE * increments
    
    # Based on current direction, uses _turn_real to turn to the given direction
    def _turn(self, new_direction: Direction):
        turnValue = int(self.current_direction) - int(new_direction)
        # If turnValue is 0, we don't need to turn
        if turnValue == 0:
            return
        # If turnValue is 2 or -2, we need to turn 180 degrees
        elif turnValue == 2 or turnValue == -2:
            self._turn_real(2)
        # If turnValue is 1 or -3, we need to turn 90 degrees
        elif turnValue == 1 or turnValue == -3:
            self._turn_real(1)
        # If turnValue is -1 or 3, we need to turn -90 degrees
        elif turnValue == -1 or turnValue == 3:
            self._turn_real(-1)
        # Set current direction to new direction
        self.current_direction = new_direction

    # Moves forward one square
    def _forward(self):
        self.left_target += self.MOVE_ROTATIONS
        self.right_target -= self.MOVE_ROTATIONS

    # Takes a direction, turns to that direction, then moves forward
    def _execute(self, direction: Direction):
        # Turn
        self._turn(direction)

        # Move
        self._forward()

    # Adds a movement action (direction) to the queue
    def add_action(self, action: Direction):
        self.action_queue.push(action)
        self.action_queue.pop()

    # Main loop
    def tick(self):
        
        # Set various variables
        left_remaining = abs(self.left_target - self.left_encoder.steps)
        right_remaining = abs(self.left_target - self.left_encoder.steps)
        left_direction = -1 if self.left_target < self.left_encoder.steps else 1
        right_direction = -1 if self.right_target < self.right_encoder.steps else 1
        left_speed = 0
        right_speed = 0

        # If reached target
        if left_remaining < self.STOPPING_ERROR and right_remaining < self.STOPPING_ERROR:
            # Reached target
            left_speed = 0
            right_speed = 0

            # Get new action from queue
            if len(self.action_queue) > 0:
                self._execute(self.action_queue[0])
                del self.action_queue[0]
            
        else:
            # Need to move
            left_speed = self.MOVE_SPEED * left_direction
            right_speed = self.MOVE_SPEED * right_direction
        
        # Modify left and right speeds if difference is greater than error
        if left_remaining < right_remaining - self.DIFFERENCE_ERROR:
            left_speed *= self.CATCHUP_MODIFIER
        elif right_remaining < left_remaining - self.DIFFERENCE_ERROR:
            right_speed *= self.CATCHUP_MODIFIER

        # Set motor speeds
        if self.left_target > self.left_encoder.steps:
            self.left_motor.forward(left_speed)
        elif self.left_target < self.left_encoder.steps:
            self.left_motor.backward(left_speed)
        else:
            self.left_motor.stop()
        if self.right_target > self.right_encoder.steps:
            self.right_motor.forward(right_speed)
        elif self.right_target < self.right_encoder.steps:
            self.right_motor.backward(right_speed)
        else:
            self.right_motor.stop()
        
        # in the drive straight funtion we want to check if one wheel has gone further than the other
        # one wheel has gone further than the other we should increase the power of the other motor so the robot drives straight
        # then should have the robot drive with the new powers for both wheels
            
        # Set motors based on drive mode


def main():
    # For now taking WASD input and feeding it into the motor module
    # Later will deal with messages from other modules

    # Create motor module
    motor_module = MotorModule(ADDRESS, PORT)
    motor_module.run()

    # Get WASD input and add the actions to the queue based on that
    while True:
        key = input()
        if key == "w":
            motor_module.add_action(Direction.FORWARD)
        elif key == "a":
            motor_module.add_action(Direction.LEFT)
        elif key == "s":
            motor_module.add_action(Direction.BACKWARD)
        elif key == "d":
            motor_module.add_action(Direction.RIGHT)
        elif key == "q":
            break


if __name__ == "__main__":
    main()
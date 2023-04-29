from enum import IntEnum
from simple_pid import PID
import time

class SimulatedMotor():
    def __init__(self, encoder):
        self.encoder = encoder
        self.speed = 0
    
    def stop(self):
        self.speed = 0

    def forward(self, amount):
        if amount < 0 or amount > 1:
            raise RuntimeError("Forward amount out of bounds")
        self.speed = amount

    def backward(self, amount):
        if amount < 0 or amount > 1:
            raise RuntimeError("Forward amount out of bounds")
        self.speed = -1*amount
    
    def move(self):
        self.encoder.move(self.speed)



class SimulatedEncoder():
    def __init__(self):
        self.steps = 0
    
    def read(self):
        return self.steps
    
    def move(self, amount):
        self.steps += amount

Direction = IntEnum("Direction", ["W", "A", "S", "D"])

class SimulatedMotorModule():
    def __init__(self):
        self.MOVE_SPEED = 0.5
        self.STOPPING_ERROR = 1
        self.DIFFERENCE_ERROR = 1
        self.CATCHUP_MODIFIER = 1.1
        self.MOVE_ROTATIONS = 10
        self.TURN_DISTANCE = 1
        self.SLOWING_ERROR = 5
        self.SLOWING_MODIFIER = 0.2
        self.FREQUENCY = 100
        self.MOVE_MODIFIER = 1.0

        self.left_encoder = SimulatedEncoder()
        self.right_encoder = SimulatedEncoder()
        self.left_motor = SimulatedMotor(self.left_encoder)
        self.right_motor = SimulatedMotor(self.right_encoder)

        self.PID_FORWARD_CONSTANTS = (0.1, 0.0, 0.0)
        self.left_pid = PID(*self.PID_FORWARD_CONSTANTS)
        self.right_pid = PID(*self.PID_FORWARD_CONSTANTS)
        self.left_pid.sample_time = 1 / self.FREQUENCY
        self.right_pid.sample_time = 1 / self.FREQUENCY
        self.left_pid.output_limits = (-1 / self.MOVE_MODIFIER, 1 / self.MOVE_MODIFIER)
        self.right_pid.output_limits = (-1 / self.MOVE_MODIFIER, 1 / self.MOVE_MODIFIER)

        self.left_pid.setpoint = 10
        self.right_pid.setpoint = 10

        WASD_to_Direction = {
            "w": Direction.W,
            "a": Direction.A,
            "s": Direction.S,
            "d": Direction.D
        }
        self.action_queue = [WASD_to_Direction[action] for action in input("Enter actions: ").split(" ")]

        self.done = False
        self.current_direction = Direction.W


    # Turns to the the increments * 90 degrees
    def _turn_real(self, increments: int):
        print("Decided to turn", increments, "times")
        self.left_target += self.TURN_DISTANCE * increments
        self.right_target += self.TURN_DISTANCE * increments
    
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
        self.right_target += self.MOVE_ROTATIONS

    # Takes a direction, turns to that direction, then moves forward
    def _execute(self, direction: Direction):
        # Turn
        # self._turn(direction)

        # Move
        self._forward()

    def tick(self):

        left_remaining = abs(self.left_pid.setpoint - self.left_encoder.read())
        right_remaining = abs(self.right_pid.setpoint - self.right_encoder.read())
            
        # Get speed from PID
        left_speed = self.left_pid(self.left_encoder.read())
        right_speed = self.right_pid(self.right_encoder.read())

        # Modify left and right speeds if difference is greater than error
        # if left_remaining < right_remaining - self.DIFFERENCE_ERROR:
        #     left_speed *= self.CATCHUP_MODIFIER
        # elif right_remaining < left_remaining - self.DIFFERENCE_ERROR:
        #     right_speed *= self.CATCHUP_MODIFIER

        print(f"   Left: target {str(round(self.left_pid.setpoint, 2)).rjust(5)} | current {str(round(self.left_encoder.read(), 2)).rjust(5)} | speed {str(round(left_speed, 2)).rjust(5)}  ===  Right: target {str(round(self.left_pid.setpoint, 2)).rjust(5)} | current {str(round(self.right_encoder.read(), 2)).rjust(5)} | speed {str(round(right_speed, 2)).rjust(5)}")

        # Set motor movement based on speed
        if left_speed == 0:
            self.left_motor.stop()
        elif left_speed > 0:
            self.left_motor.forward(left_speed)
        else:
            self.left_motor.backward(-left_speed)
        if right_speed == 0:
            self.right_motor.stop()
        elif right_speed > 0:
            self.right_motor.forward(right_speed)
        else:
            self.right_motor.backward(-right_speed)
        
        self.left_motor.move()
        self.right_motor.move()
    

    

def main():
    mm = SimulatedMotorModule()
    while not mm.done:
        mm.tick()
        time.sleep(1 / mm.FREQUENCY)

if __name__ == "__main__":
    main()
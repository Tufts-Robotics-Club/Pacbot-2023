from enum import IntEnum

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
    
    def move(self, amount):
        self.steps += amount

Direction = IntEnum("Direction", ["W", "A", "S", "D"])

class SimulatedMotorModule():
    def __init__(self):
        self.MOVE_SPEED = 0.5
        self.STOPPING_ERROR = 1
        self.DIFFERENCE_ERROR = 1
        self.CATCHUP_MODIFIER = 1.1
        self.MOVE_ROTATIONS = 1
        self.TURN_DISTANCE = 1

        self.left_encoder = SimulatedEncoder()
        self.right_encoder = SimulatedEncoder()
        self.left_motor = SimulatedMotor(self.left_encoder)
        self.right_motor = SimulatedMotor(self.right_encoder)

        self.left_target = 0
        self.right_target = 0

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
        self.right_target -= self.MOVE_ROTATIONS

    # Takes a direction, turns to that direction, then moves forward
    def _execute(self, direction: Direction):
        # Turn
        self._turn(direction)

        # Move
        self._forward()

    def tick(self):
        left_remaining = abs(self.left_target - self.left_encoder.steps)
        right_remaining = abs(self.right_target - self.right_encoder.steps)
        left_speed = 0
        right_speed = 0


        # Checking target reached independently
        if left_remaining < self.STOPPING_ERROR:
            # Reached target
            left_speed = 0
        else:
            left_speed = self.MOVE_SPEED
        if right_remaining < self.STOPPING_ERROR:
            right_speed = 0
        else:
            right_speed = self.MOVE_SPEED
        
        # If reached target (both)
        if left_remaining < self.STOPPING_ERROR and right_remaining < self.STOPPING_ERROR:
            # Get new action from queue
            if len(self.action_queue) > 0:
                print("Executing action:", self.action_queue[0])
                self._execute(self.action_queue[0])
                del self.action_queue[0]
            else:
                self.done = True

        
        # Modify left and right speeds if difference is greater than error
        if left_remaining < right_remaining - self.DIFFERENCE_ERROR:
            left_speed *= self.CATCHUP_MODIFIER
        elif right_remaining < left_remaining - self.DIFFERENCE_ERROR:
            right_speed *= self.CATCHUP_MODIFIER

        print(f"   Left: target {str(self.left_target).rjust(5)} | current {str(self.left_encoder.steps).rjust(5)} | speed {str(left_speed).rjust(5)}  ===  Right: target {str(self.right_target).rjust(5)} | current {str(self.right_encoder.steps).rjust(5)} | speed {str(right_speed).rjust(5)}")

        # Set motor movement based on speed
        if left_speed == 0:
            self.left_motor.stop()
        elif self.left_target > self.left_encoder.steps:
            self.left_motor.forward(left_speed)
        else:
            self.left_motor.backward(left_speed)
        if right_speed == 0:
            self.right_motor.stop()
        elif self.right_target > self.right_encoder.steps:
            self.right_motor.forward(right_speed)
        else:
            self.right_motor.backward(right_speed)
        
        self.left_motor.move()
        self.right_motor.move()
    

    

def main():
    mm = SimulatedMotorModule()
    while not mm.done:
        mm.tick()

if __name__ == "__main__":
    main()
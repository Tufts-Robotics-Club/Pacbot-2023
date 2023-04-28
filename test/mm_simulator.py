
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

class SimulatedMotorModule():
    def __init__(self):
        self.MOVE_SPEED = 0.5
        self.STOPPING_ERROR = 1
        self.DIFFERENCE_ERROR = 1
        self.CATCHUP_MODIFIER = 1.1

        self.left_encoder = SimulatedEncoder()
        self.right_encoder = SimulatedEncoder()
        self.left_motor = SimulatedMotor(self.left_encoder)
        self.right_motor = SimulatedMotor(self.right_encoder)

        self.left_target = int(input("Left target: "))
        self.right_target = int(input("Right target: "))

        print("HEY")
        self.done = False

    def tick(self):
    # Set various variables
        print("Left:", self.left_target, self.left_encoder.steps)
        print("Right:", self.right_target, self.right_encoder.steps)
        left_remaining = abs(self.left_target - self.left_encoder.steps)
        right_remaining = abs(self.right_target - self.right_encoder.steps)
        left_speed = 0
        right_speed = 0


        # Checking target reached independently
        if left_remaining < self.STOPPING_ERROR:
            # Reached target
            left_speed = 0
        if right_remaining < self.STOPPING_ERROR:
            right_speed = 0
        
        # If reached target (both)
        if left_remaining < self.STOPPING_ERROR and right_remaining < self.STOPPING_ERROR:
            # Get new action from queue
            if len(self.action_queue) > 0:
                self._execute(self.action_queue[0])
                del self.action_queue[0]
            self.done = True
        else:
            # Need to move
            left_speed = self.MOVE_SPEED
            right_speed = self.MOVE_SPEED
        
        # Modify left and right speeds if difference is greater than error
        if left_remaining < right_remaining - self.DIFFERENCE_ERROR:
            left_speed *= self.CATCHUP_MODIFIER
        elif right_remaining < left_remaining - self.DIFFERENCE_ERROR:
            right_speed *= self.CATCHUP_MODIFIER

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
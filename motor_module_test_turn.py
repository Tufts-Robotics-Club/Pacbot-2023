import robomodules as rm
from messages import MsgType, message_buffers, PacmanDirection, GyroYaw
from gpiozero import PhaseEnableMotor, RotaryEncoder
from enum import IntEnum
import math
import os
from simple_pid import PID
import board
import adafruit_bno055
import adafruit_tca9548a

ADDRESS = os.environ.get("LOCAL_ADDRESS", "localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

Direction = IntEnum("Direction", ["W", "A", "S", "D"])
Mode = IntEnum("Mode", ["forward", "turn", "stop"])

class MotorModule(rm.ProtoModule):
    def __init__(self, addr, port):
        
        # Constants \U+1F929
        self.FREQUENCY = 100

        # How far from the target distance is acceptible before stopping
        self.STOPPING_ERROR = 1
        self.TURN_ERROR = 1
        # How much the two wheels can be different before we try to compensate
        self.DIFFERENCE_ERROR = 0.1
        
        self.MOVE_MODIFIER = 1.0
        self.TURN_MODIFIER = 1.0
        # 6 inches / (Wheel diameter * pi * 1 in / 25.2 mm)
        self.MOVE_ROTATIONS = 10 # 6 / (32 * math.pi / 25.4)
        self.CATCHUP_MODIFIER = 1.1
        self.LEFT_MOTOR_PINS = (27, 22)
        self.RIGHT_MOTOR_PINS = (5, 6)
        self.LEFT_ENCODER_PINS = (23, 24)
        self.RIGHT_ENCODER_PINS = (14, 15)
        self.PID_FORWARD_CONSTANTS = (1.0, 0.1, 0.05)
        self.PID_TURN_CONSTANTS = (0.008, 0, 0)

        # PIDs
        self.left_pid = PID(*self.PID_FORWARD_CONSTANTS)
        self.right_pid = PID(*self.PID_FORWARD_CONSTANTS)
        self.turn_pid = PID(*self.PID_TURN_CONSTANTS)
        self.left_pid.sample_time = 1 / self.FREQUENCY
        self.right_pid.sample_time = 1 / self.FREQUENCY
        self.turn_pid.sample_time = 1 / self.FREQUENCY
        self.left_pid.output_limits = (-1 / self.MOVE_MODIFIER, 1 / self.MOVE_MODIFIER)
        self.right_pid.output_limits = (-1 / self.MOVE_MODIFIER, 1 / self.MOVE_MODIFIER)
        self.turn_pid.output_limits = (-1 / self.TURN_MODIFIER, 1 / self.TURN_MODIFIER)

        # Need to set up connections and stuff
        self.subscriptions = [MsgType.PACMAN_DIRECTION]
        super().__init__(addr, port, message_buffers, MsgType, self.FREQUENCY, self.subscriptions)

        # Motors - have to change the pins or whatever
        self.left_motor = PhaseEnableMotor(*self.LEFT_MOTOR_PINS)
        self.right_motor = PhaseEnableMotor(*self.RIGHT_MOTOR_PINS)
        # Encoders
        self.left_encoder = RotaryEncoder(*self.LEFT_ENCODER_PINS, max_steps=0)
        self.right_encoder = RotaryEncoder(*self.RIGHT_ENCODER_PINS, max_steps=0)
        # Gyro
        i2c = board.I2C()
        tca = adafruit_tca9548a.TCA9548A(i2c)
        self.sensor = adafruit_bno055.BNO055_I2C(tca[1])

        self.left_pid.setpoint = self.left_encoder.steps
        self.right_pid.setpoint = self.right_encoder.steps
        self.turn_countdown = 100
        self.initial_turn_set = False

        self.current_direction = Direction.W
        self.mode = Mode.stop
        
    
    # Based on current direction, uses _turn_real to turn to the given direction
    def _turn(self, new_direction: Direction):
        self.turn_pid.reset()
        turnValue = int(self.current_direction) - int(new_direction)
        # If turnValue is 0, we don't need to turn
        if turnValue == 0:
            return
        # If turnValue is 2 or -2, we need to turn 180 degrees
        elif turnValue == 2 or turnValue == -2:
            self.turn_pid.setpoint += 2 * 90
        # If turnValue is 1 or -3, we need to turn 90 degrees
        elif turnValue == 1 or turnValue == -3:
            self.turn_pid.setpoint += 1 * 90
        # If turnValue is -1 or 3, we need to turn -90 degrees
        elif turnValue == -1 or turnValue == 3:
            self.turn_pid.setpoint += -1 * 90
        # Set current direction to new direction
        self.current_direction = new_direction

    # Moves forward one square
    def _forward(self):
        self.left_pid.setpoint += self.MOVE_ROTATIONS
        self.right_pid.setpoint -= self.MOVE_ROTATIONS

    # Takes a direction, turns to that direction, then moves forward
    def _execute(self, direction: Direction):
        self.left_pid.reset()
        self.right_pid.reset()
        self.mode = Mode.turn
        # Turn
        self._turn(direction)

        # Move
        self._forward()

    # Adds a movement action (direction) to the queue
    def add_action(self, action: Direction):
        self.action_queue.append(action)

    # Main loop
    def tick(self):
        left_remaining = abs(self.left_pid.setpoint - self.left_encoder.steps)
        right_remaining = abs(self.right_pid.setpoint - self.right_encoder.steps)
        left_speed = 0
        right_speed = 0

        # If reached target (both)
        if left_remaining < self.STOPPING_ERROR and right_remaining < self.STOPPING_ERROR:
            self.mode = Mode.stop
        
        if self.mode == Mode.stop:
            # Setting starting angle in init gets mad :(
            start_angle = self.sensor.euler[0]
            if start_angle >= 0 and self.turn_countdown <= 0 and not self.initial_turn_set:
                self.turn_pid.setpoint = start_angle
                self.initial_turn_set = True
            self.turn_countdown -= 1
            print(self.turn_pid.setpoint)
            self.left_motor.stop()
            self.right_motor.stop()
        elif self.mode == Mode.turn:
            angle = self.sensor.euler[0]

            # If close enough, stop turning
            if abs(angle - self.turn_pid.setpoint) < self.TURN_ERROR:
                self.mode = Mode.stop
                self.left_motor.stop()
                self.right_motor.stop()
                return

            # Discard bad sensor values
            if angle < 0:
                print("Bad sensor value")
                return

            # Change PID target if too far away
            if angle > self.turn_pid.setpoint + 180:
                self.turn_pid.setpoint += 360
            elif angle < self.turn_pid.setpoint - 180:
                self.turn_pid.setpoint -= 360

            # Get speed from PID
            speed = self.turn_pid(angle)

            print(f"Target: {self.turn_pid.setpoint} | Current: {angle} | Speed: {speed}")

            if speed < 0:
                self.left_motor.forward(-speed)
                self.right_motor.backward(-speed)
            else:
                self.left_motor.backward(speed)
                self.right_motor.forward(speed)
            
        elif self.mode == Mode.forward:
            # Get speed from PID
            left_speed = self.left_pid(self.left_encoder.steps) * self.MOVE_MODIFIER
            right_speed = self.right_pid(self.right_encoder.steps) * self.MOVE_MODIFIER

            # Modify left and right speeds if difference is greater than error
            if left_remaining < right_remaining - self.DIFFERENCE_ERROR:
                left_speed *= self.CATCHUP_MODIFIER
            elif right_remaining < left_remaining - self.DIFFERENCE_ERROR:
                right_speed *= self.CATCHUP_MODIFIER

            # print(f"   Left: target {str(self.left_pid.setpoint).rjust(5)} | current {str(self.left_encoder.steps).rjust(5)} | speed {str(left_speed).rjust(5)}  ===  Right: target {str(self.left_pid.setpoint).rjust(5)} | current {str(self.right_encoder.steps).rjust(5)} | speed {str(right_speed).rjust(5)}")

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
        
        
        # in the drive straight funtion we want to check if one wheel has gone further than the other
        # one wheel has gone further than the other we should increase the power of the other motor so the robot drives straight
        # then should have the robot drive with the new powers for both wheels
            
        # Set motors based on drive mode

    def msg_received(self, msg, msg_type):
        # Takes the message and adds it to the queue
        if self.mode == Mode.stop and msg_type == MsgType.PACMAN_DIRECTION:
            print("Message received!")
            self._execute(msg.direction)


def main():

    # Create motor module
    motor_module = MotorModule(ADDRESS, PORT)
    motor_module.run()


if __name__ == "__main__":
    main()
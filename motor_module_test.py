import robomodules as rm
from messages import MsgType, message_buffers, PacmanDirection, GyroYaw
from gpiozero import PhaseEnableMotor
import Encoder
from enum import IntEnum
import math
import os
from simple_pid import PID
import board
import adafruit_bno055
import adafruit_tca9548a
import time
from distance_sensors import DistanceSensor


ADDRESS = os.environ.get("LOCAL_ADDRESS", "localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

Direction = IntEnum("Direction", ["W", "A", "S", "D"])
Mode = IntEnum("Mode", ["forward", "turn", "stop"])

BACKUP_SPEED = 0.5

######### IMU STUFF START #$#####

i2c = board.I2C()  # uses board.SCL and board.SDA

# multiplexer
tca = adafruit_tca9548a.TCA9548A(i2c)

# imu (attached to port 0 of multiplexer)
sensor = adafruit_bno055.BNO055_I2C(tca[0])
sensors = DistanceSensor(":3", tca[3], 8)

# Default calibration stuff (add me back in to skip)
sensor.offsets_gyroscope = (-2, -1, 129)
sensor.offsets_accelerometer = (-23, -42, -31)
sensor.offsets_magnetometer = (-157, 732, 337)

TARGET_CALIBRATION_VAL = 3

done = False
while True:
    calibration_status = sensor.calibration_status
    print("Calibration Status (system, gyro, accel, mag): {}".format(calibration_status), "Euler Angle: {}".format(sensor.euler))

    if calibration_status[1] == TARGET_CALIBRATION_VAL and calibration_status[1] == TARGET_CALIBRATION_VAL and calibration_status[2] == TARGET_CALIBRATION_VAL and calibration_status[3] == TARGET_CALIBRATION_VAL:
        print("Target reached!")
        break

    time.sleep(0.01)



print("final calibration vals (gyro, accel, mag)", sensor.offsets_gyroscope, sensor.offsets_accelerometer, sensor.offsets_magnetometer)


######### IMU STUFF END #$#####



class MotorModule(rm.ProtoModule):
    def __init__(self, addr, port):
        
        # Constants \U+1F929
        self.FREQUENCY = 100

        # How far from the target distance is acceptible before stopping
        self.STOPPING_ERROR = 5
        self.TURN_ERROR = 1.5
        # How much the two wheels can be different before we try to compensate
        self.DIFFERENCE_ERROR = 0.1
        self.following = False
        self.MOVE_MODIFIER = 1.0
        self.TURN_MODIFIER = 1.0
        self.MOVE_ROTATIONS = 200
        self.CATCHUP_MODIFIER = 1.1
        self.LEFT_MOTOR_PINS = (22, 27)
        self.RIGHT_MOTOR_PINS = (6, 5)
        self.LEFT_ENCODER_PINS = (23, 24)
        self.RIGHT_ENCODER_PINS = (17, 25)
        self.PID_FORWARD_CONSTANTS = (0.005, 0.0, 0.0)
        self.PID_TURN_CONSTANTS = (0.015, 0, 0)

        # PIDs
        self.left_pid = PID(*self.PID_FORWARD_CONSTANTS)
        self.right_pid = PID(*self.PID_FORWARD_CONSTANTS)
        self.turn_pid = PID(*self.PID_TURN_CONSTANTS)
        self.left_pid.sample_time = 1 / self.FREQUENCY
        self.right_pid.sample_time = 1 / self.FREQUENCY
        self.turn_pid.sample_time = 1 / self.FREQUENCY
        self.left_pid.output_limits = (-0.6 / self.CATCHUP_MODIFIER, 0.6 / self.CATCHUP_MODIFIER)
        # self.right_pid.output_limits = (-1, 1)
        # self.left_pid.output_limits = (-1, 1)
        self.right_pid.output_limits = (-0.6 / self.CATCHUP_MODIFIER, 0.6 / self.CATCHUP_MODIFIER)
        self.turn_pid.output_limits = (-1 / self.TURN_MODIFIER, 1 / self.TURN_MODIFIER)

        # Need to set up connections and stuff
        self.subscriptions = [MsgType.PACMAN_DIRECTION, MsgType.GYRO_YAW]
        super().__init__(addr, port, message_buffers, MsgType, self.FREQUENCY, self.subscriptions)

        # Motors - have to change the pins or whatever
        self.left_motor = PhaseEnableMotor(*self.LEFT_MOTOR_PINS)
        self.right_motor = PhaseEnableMotor(*self.RIGHT_MOTOR_PINS)
        # Encoders
        self.left_encoder = Encoder.Encoder(*self.LEFT_ENCODER_PINS)
        self.right_encoder = Encoder.Encoder(*self.RIGHT_ENCODER_PINS)
        # Gyro
        i2c = board.I2C()
        tca = adafruit_tca9548a.TCA9548A(i2c)
        self.sensor = adafruit_bno055.BNO055_I2C(tca[1])
        self.sensor.offsets_gyroscope = (-1, 129, 0)
        self.sensor.offsets_accelerometer = (3, -52, -8)
        self.sensor.offsets_magnetometer = (54, 773, 286) 

        self.left_pid.setpoint = -self.left_encoder.read()
        self.right_pid.setpoint = self.right_encoder.read()
        self.turn_countdown = 100
        self.initial_turn_set = False

        self.current_direction = 3
        self.mode = Mode.stop

        self.struggle = 200 # freq = 100
        self.abort_count = 0
        self.wait_count = 3
        
    
    # Based on current direction, uses _turn_real to turn to the given direction
    def _turn(self, new_direction: Direction):
        print("currently at", int(self.current_direction), "turning to", new_direction)
        # self.turn_pid.reset()
        turnValue = int(self.current_direction) - int(new_direction)
        # If turnValue is 0, we don't need to turn
        if turnValue == 0:
            print("no need to turn")
            return
        # If turnValue is 2 or -2, we need to turn 180 degrees
        elif turnValue == 2 or turnValue == -2:
            print("turning 180 degrees")
            self.turn_pid.setpoint += 2 * 90
        # If turnValue is 1 or -3, we need to turn 90 degrees
        elif turnValue == 1 or turnValue == -3:
            print("turning 90 degrees")
            self.turn_pid.setpoint += 1 * 90
        # If turnValue is -1 or 3, we need to turn -90 degrees
        elif turnValue == -1 or turnValue == 3:
            print("turning -90 degrees")
            self.turn_pid.setpoint += -1 * 90
        # Set current direction to new direction
        self.current_direction = new_direction

    # Moves forward one square
    def _forward(self):
        # self.left_pid.setpoint -= self.MOVE_ROTATIONS
        # self.right_pid.setpoint -= self.MOVE_ROTATIONS
        self.left_pid.setpoint = -self.left_encoder.read() - self.MOVE_ROTATIONS
        self.right_pid.setpoint = self.right_encoder.read() - self.MOVE_ROTATIONS
        print("aiming left encoder for", self.left_pid.setpoint, "and right for", self.right_pid.setpoint)

    # Takes a direction, turns to that direction, then moves forward
    def _execute(self, direction: Direction):
        self.following = True
        self.mode = Mode.turn
        # Turn
        self._turn(direction)
        
        self.left_pid.reset()
        self.right_pid.reset()
        # Move
        # self._forward()

    # Adds a movement action (direction) to the queue
    def add_action(self, action: Direction):
        self.action_queue.append(action)

    # Main loop
    def tick(self):
        #if self.initial_turn_set == False:
        #    print("initlai turn set flase")
        if self.mode == Mode.stop:
            self.following = False
            # print("in stop mode yeaaaa")
            # Setting starting angle in init gets mad :(
            start_angle = self.sensor.euler[0]
            if start_angle is not None and start_angle >= 0 and not self.initial_turn_set and self.turn_countdown <= 0:
                self.turn_pid.setpoint = start_angle
                self.initial_turn_set = True
                print("initialized start angle to", start_angle)
            self.turn_countdown -= 1
            self.wait_count -= 1
            self.left_motor.stop()
            self.right_motor.stop()
        elif self.wait_count > 0:
            self.wait_count -= 1
            return
        
        if self.mode == Mode.turn:
            angle = self.sensor.euler[0]

            # If close enough, stop turning
            if abs(angle - self.turn_pid.setpoint) < self.TURN_ERROR:
                self.mode = Mode.forward
                self._forward()
                self.left_motor.stop()
                self.right_motor.stop()
                self.wait_count = 3
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
            if sensors.is_too_close(): # if front is blocked, stop :3
                print("I HAVE BEEN OBSTRUCTED :3")
                self.left_motor.stop()
                self.right_motor.stop()
                self.mode = Mode.stop
                self.struggle = 200
            left_remaining = abs(self.left_pid.setpoint - -self.left_encoder.read())
            right_remaining = abs(self.right_pid.setpoint - self.right_encoder.read())
            self.struggle -= 1

            if self.struggle <= 0:
                self.abort_count = 25
                self.struggle = 200

            if self.abort_count > 0:
                # self.dir_blocked = sensors.which_obstructed()
                # print("bein sneaky")
                # ### SUPER SNEAKY ABORT CODE
                # if (self.dir_blocked is not None):
                print("I got stuck :(")
                self.left_motor.forward(BACKUP_SPEED)
                self.right_motor.forward(BACKUP_SPEED)
                # if (self.dir_blocked == 3): # front obstructed
                #     print("front sneak")
                #     self.left_motor.backward(BACKUP_SPEED)
                #     self.right_motor.backward(BACKUP_SPEED)
                # if (self.dir_blocked == 2): # left obstructed, assuming we're going forward
                #     print("left sneak")
                #     self.left_motor.backward(BACKUP_SPEED)
                #     self.right_motor.backward(BACKUP_SPEED/2)
                #     self.abort_count -= 1
                # if (self.dir_blocked == 0): #right obstructed, assuming we're going forward
                #     print("right sneak")
                #     self.left_motor.backward(BACKUP_SPEED)
                #     self.right_motor.backward(BACKUP_SPEED/2)
                #     self.abort_count -= 10
                # if (self.dir_blocked == 1): #back... obstructed? so just... forward
                #     print("back sneak")
                #     self.left_motor.forward(BACKUP_SPEED)
                #     self.right_motor.forward(BACKUP_SPEED)
                self.abort_count -= 1
                
                if self.abort_count == 1:
                    self.left_motor.stop()
                    self.right_motor.stop()
                    self.mode = Mode.stop
                    self.struggle = 250
                    self.wait_count = 3
            else: # proceed as normal, not obstructed
                # If reached target (both)
                # print("vibes")
                if left_remaining < self.STOPPING_ERROR and right_remaining < self.STOPPING_ERROR:
                    self.left_motor.stop()
                    self.right_motor.stop()
                    self.mode = Mode.stop
                    self.struggle = 250
                    self.wait_count = 3

                # Get speed from PID
                left_speed = self.left_pid(-self.left_encoder.read()) * self.MOVE_MODIFIER
                right_speed = self.right_pid(self.right_encoder.read()) * self.MOVE_MODIFIER

                # Modify left and right speeds if difference is greater than error
                if left_remaining < right_remaining - self.DIFFERENCE_ERROR:
                    left_speed *= self.CATCHUP_MODIFIER
                elif right_remaining < left_remaining - self.DIFFERENCE_ERROR:
                    right_speed *= self.CATCHUP_MODIFIER

                print(f"   Left: target {str(self.left_pid.setpoint).rjust(5)} | current {str(-self.left_encoder.read()).rjust(5)} | speed {str(left_speed).rjust(5)}  ===  Right: target {str(self.right_pid.setpoint).rjust(5)} | current {str(self.right_encoder.read()).rjust(5)} | speed {str(right_speed).rjust(5)}")

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
            print("Message received!", msg.direction)
            if msg.direction == PacmanDirection.STOP:
                self.mode = Mode.stop
            elif not self.following: 
                self._execute(msg.direction)
        if msg_type == MsgType.GYRO_YAW:
            print("zeroing gyro")
            self.initial_turn_set = False
            self.current_direction = 3


def main():

    # Create motor module
    motor_module = MotorModule(ADDRESS, PORT)
    motor_module.run()


if __name__ == "__main__":
    main()


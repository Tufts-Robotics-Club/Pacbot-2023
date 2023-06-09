from gpiozero import PhaseEnableMotor #, RotaryEncoder
from time import sleep
import time
import board
import adafruit_bno055
import adafruit_tca9548a
from simple_pid import PID

i2c = board.I2C()  # uses board.SCL and board.SDA

# multiplexer
tca = adafruit_tca9548a.TCA9548A(i2c)

# imu (attached to port 0 of multiplexer)
sensor = adafruit_bno055.BNO055_I2C(tca[1])

motor1 = PhaseEnableMotor(27, 22)
motor2 = PhaseEnableMotor(5, 6)
# rotar1 = RotaryEncoder(23, 24, max_steps=0)
# rotar2 = RotaryEncoder(14, 15, max_steps=0)



def turn90():
    print("Start Euler angle: {}".format(sensor.euler))

    MAX_ANGLE = 360
    TURN_ANGLE = 90

    START_ANGLE = sensor.euler[0] #ideally 0, often 359
    # TARGET_ANGLE = round(START_ANGLE + TURN_ANGLE)
    #     if TARGET_ANGLE > MAX_ANGLE:
    #         TARGET_ANGLE = MAX_ANGLE
    #     elif TARGET_ANGLE < -MAX_ANGLE:
    #         TARGET_ANGLE = -MAX_ANGLE

    if START_ANGLE < 0:
            START_ANGLE = 0
    elif START_ANGLE > 360:
            START_ANGLE = 360

    TARGET_ANGLE = (START_ANGLE + TURN_ANGLE) % 360 #90

    print("Target angle: "+ str(TARGET_ANGLE))

    OFFSET = 0
    if (START_ANGLE > TARGET_ANGLE):
        OFFSET = 360 - START_ANGLE

    # DELTA = START_ANGLE - OFFSET

    TARGET_ANGLE = TARGET_ANGLE + OFFSET

    pid = PID(0.008, 0, 0, setpoint=TARGET_ANGLE) #90
    pid.output_limits = (-1, 1)

    range_upper = TARGET_ANGLE + 2
    range_lower = TARGET_ANGLE - 2
    print("Range: "+ str(range_lower) + ", " + str(range_upper))

    while True:
        current_angle = (sensor.euler[0] + OFFSET) % 360
        if current_angle is None:
            print("none :(")
        elif current_angle < 0:
            continue
    # elif current_angle > 360:
    #     current_angle = current_angle
        else:
            if current_angle < TARGET_ANGLE + 2 and current_angle > TARGET_ANGLE - 2:
                print("I am in range!", current_angle)
                motor1.stop()
                motor2.stop()
                sleep(0.1)
                print("Stop motor")
                break
            control = pid(current_angle)
            print("control:", control, "angle:", current_angle, "target:", TARGET_ANGLE)
            if control < 0:
                motor2.backward(-control)
                motor1.forward(-control)
            else:
                motor2.forward(control)
                motor1.backward(control)
        sleep(0.1)


        # if (sensor.euler[0] > 180):
        #     DELTA = sensor.euler[0] - OFFSET
        # else:
        #     DELTA = sensor.euler[0]

        # print("Current Euler angle: {}".format(sensor.euler))
        # print("Delta: " + str(DELTA))

    # motor1.stop()
    # motor2.stop()

    #     print(rotar1.steps, rotar2.steps)


for i in range(0, 4):
    print("I'm going on ", i)
    turn90()

from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep
import time
import board
import adafruit_bno055
import adafruit_tca9548a

i2c = board.I2C()  # uses board.SCL and board.SDA

# multiplexer
tca = adafruit_tca9548a.TCA9548A(i2c)

# imu (attached to port 0 of multiplexer)
sensor = adafruit_bno055.BNO055_I2C(tca[0])

motor1 = PhaseEnableMotor(27, 22)
motor2 = PhaseEnableMotor(5, 6)
rotar1 = RotaryEncoder(23, 24, max_steps=0)
rotar2 = RotaryEncoder(14, 15, max_steps=0)

print("Start Euler angle: {}".format(sensor.euler))  

MAX_ANGLE = 360
TURN_ANGLE = 90

START_ANGLE = sensor.euler[0] #ideally 0, often 359
# TARGET_ANGLE = round(START_ANGLE + TURN_ANGLE)
#     if TARGET_ANGLE > MAX_ANGLE:
#         TARGET_ANGLE = MAX_ANGLE
#     elif TARGET_ANGLE < -MAX_ANGLE:
#         TARGET_ANGLE = -MAX_ANGLE

TARGET_ANGLE = (START_ANGLE + TURN_ANGLE) % 360 #90
print("Target angle: "+ str(TARGET_ANGLE))       

OFFSET = 0
if (START_ANGLE > 90):
    OFFSET = 360

DELTA = START_ANGLE - OFFSET
#DELTA = 0 to -1

motor2.backward(0.1)
motor1.forward(0.1)

# TODO: Both motors are going forward, need to have one going backwards
#       Not stopping at 90 if START_ANGLE is ~360 due to inaccurate readings
while (DELTA < TARGET_ANGLE):
    if (sensor.euler[0] > 180):
        DELTA = sensor.euler[0] - OFFSET
    else:
        DELTA = sensor.euler[0]

    print("Current Euler angle: {}".format(sensor.euler))      
    print("Delta: " + str(DELTA))        
    
    

motor1.stop()
motor2.stop()

#     print(rotar1.steps, rotar2.steps)


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

motor1 = PhaseEnableMotor(19, 26)
motor2 = PhaseEnableMotor(5, 6)
rotar1 = RotaryEncoder(23, 24, max_steps=0)
rotar2 = RotaryEncoder(14, 15, max_steps=0)

TURN_ANGLE = 90
START_ANGLE = sensor.euler[0]
print("Start Euler angle: {}".format(sensor.euler))       
TARGET_ANGLE = (START_ANGLE + TURN_ANGLE) % 360
print(TARGET_ANGLE)       

while (sensor.euler[0] < TARGET_ANGLE):
    print("Current Euler angle: {}".format(sensor.euler))       
    motor2.forward(0.5)
    motor1.backward(0.5)
    sleep(0.1)

motor1.stop()
motor2.stop()

#     print(rotar1.steps, rotar2.steps)


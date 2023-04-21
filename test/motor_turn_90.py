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


MAX_ANGLE = 90
START_ANGLE = sensor.euler[0]
print("Euler angle: {}".format(sensor.euler))       
TARGET_ANGLE = sensor.euler[0] + MAX_ANGLE
print(TARGET_ANGLE)       

motor1 = PhaseEnableMotor(19, 26)
motor2 = PhaseEnableMotor(5, 6)
rotar1 = RotaryEncoder(23, 24, max_steps=0)
rotar2 = RotaryEncoder(14, 15, max_steps=0)

while (sensor.euler[0] < TARGET_ANGLE):
    print("Euler angle: {}".format(sensor.euler))       
    motor1.forward(0.5)
    motor2.backward(0.5)
    sleep(0.1)

motor1.stop()
motor2.stop()

#     print(rotar1.steps, rotar2.steps)


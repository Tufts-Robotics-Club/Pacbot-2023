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
print("Gyroscope (rad/sec): {}".format(sensor.gyro))

motor1 = PhaseEnableMotor(19, 26)
motor2 = PhaseEnableMotor(5, 6)
rotar1 = RotaryEncoder(23, 24, max_steps=0)
rotar2 = RotaryEncoder(14, 15, max_steps=0)
motor1.forward(0.5)
motor2.forward(0.5)
motor1.stop()
motor2.stop()

angle = sensor.euler[0]
print("Euler angle: {}".format(sensor.euler))      
while angle - sensor.euler[0] < 45:
    #angle += sensor.gyro[0] * 57.29578
    #angle += sensor.euler[0]
    #print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    print("Euler angle: {}".format(sensor.euler))      
    print(sensor.gyro[0])
    print(angle)
    motor1.backward(0.2)
    motor2.forward(0.2)
    sleep(0.1)


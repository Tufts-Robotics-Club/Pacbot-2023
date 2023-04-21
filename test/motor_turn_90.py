from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep

MAX_ANGLE = 90
START_ANGLE = sensor.euler[0]
TARGET_ANGLE = sensor.euler[0] + MAX_ANGLE

motor1 = PhaseEnableMotor(19, 26)
motor2 = PhaseEnableMotor(6, 13)
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


from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep

motor1 = PhaseEnableMotor(19, 26)
motor2 = PhaseEnableMotor(6, 13)
rotar1 = RotaryEncoder(23, 24, max_steps=0)
rotar2 = RotaryEncoder(14, 15, max_steps=0)
motor1.forward(0.5)
motor2.forward(0.5)
motor1.stop()
motor2.stop()

angle = 0

while angle < 90:
    angle += sensor.gyro[0]
    print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    print(rotar1.steps, rotar2.steps)
    motor1.forward(0.5)
    motor2.backward(0.5)
    sleep(0.1)


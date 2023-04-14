from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep

motor1 = PhaseEnableMotor(19, 26)
motor2 = PhaseEnableMotor(6, 13)
rotar1 = RotaryEncoder(23, 24, max_steps=12)
motor1.forward()
motor2.forward()
while True:
    print(rotar1.value)


sleep(3)

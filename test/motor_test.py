from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep

motor1 = PhaseEnableMotor(22, 27)
motor2 = PhaseEnableMotor(5, 6)
rotar1 = RotaryEncoder(23, 24, max_steps=0)
rotar2 = RotaryEncoder(14, 15, max_steps=0)
motor1.forward(0.5)
motor2.forward(0.5)

while True:
    print(rotar1.steps, rotar2.steps)
    sleep(0.1)

motor1.stop()
motor2.stop()
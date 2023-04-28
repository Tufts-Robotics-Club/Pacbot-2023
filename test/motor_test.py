from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep

motor1 = PhaseEnableMotor(27, 22)
motor2 = PhaseEnableMotor(5, 6)
rotar1 = RotaryEncoder(23, 24, max_steps=0)
rotar2 = RotaryEncoder(14, 15, max_steps=0)
# motor1.backward(0.5)
# motor2.forward(0.5)

counter = 0
while counter < 100:
    print(rotar1.steps, rotar2.steps)
    motor1.backward(0.5)
    motor2.backward(0.5)
    counter += 1
    sleep(0.05)


motor1.stop()
motor2.stop()
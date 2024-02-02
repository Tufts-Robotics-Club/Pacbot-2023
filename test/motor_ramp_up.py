from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep

motor1 = PhaseEnableMotor(22, 27)
motor2 = PhaseEnableMotor(6, 5)
rotar1 = RotaryEncoder(23, 24, max_steps=0)
rotar2 = RotaryEncoder(17, 25, max_steps=0)
speed = 0.1
motor1.forward(speed)
motor2.forward(speed)
counter = 1
while counter < 1000:
    print(rotar1.steps, rotar2.steps)
    if counter % 250 == 0:
        speed += 0.2
        print("ramping up to", speed)
        motor1.forward(speed)
        motor2.forward(speed)
    # motor1.backward(0.5)
    # motor2.backward(0.5)
    counter += 1
    sleep(0.01)


motor1.stop()
motor2.stop()

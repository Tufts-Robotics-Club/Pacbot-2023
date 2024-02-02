from gpiozero import PhaseEnableMotor, RotaryEncoder
from time import sleep

motor1 = PhaseEnableMotor(22, 27)
motor2 = PhaseEnableMotor(6, 5)
rotar1 = RotaryEncoder(23, 24, max_steps=0, bounce_time=0.005)
rotar2 = RotaryEncoder(17, 25, max_steps=0, bounce_time=0.005)
speed = 0.5
motor1.backward(speed)
motor2.backward(speed)
counter = 0
while counter < 1000:
    print(rotar1.steps, rotar2.steps)
    if counter == 500:
        motor1.stop()
        motor2.stop()
        sleep(0.01)
        print("turning around")
        motor1.forward(speed)
        motor2.forward(speed)
    # motor1.backward(0.5)
    # motor2.backward(0.5)
    counter += 1
    sleep(0.01)


motor1.stop()
motor2.stop()

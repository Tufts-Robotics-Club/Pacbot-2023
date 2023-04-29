from gpiozero import PhaseEnableMotor
import Encoder
from time import sleep

motor1 = PhaseEnableMotor(22, 27)
motor2 = PhaseEnableMotor(6, 5)
rotar1 = Encoder.Encoder(23, 24)
rotar2 = Encoder.Encoder(17, 25)
# motor1.backward(0.5)
# motor2.forward(0.5)

counter = 0
while counter < 100:
    print(rotar1.read(), rotar2.read())
    motor1.backward(0.1)
    motor2.backward(0.1)
    counter += 1
    sleep(0.05)


motor1.stop()
motor2.stop()
from gpiozero import PhaseEnableMotor
from time import sleep

motor1 = PhaseEnableMotor(19, 26)
motor2 = PhaseEnableMotor(6, 13)
motor1.forward()
motor2.forward()

sleep(3)

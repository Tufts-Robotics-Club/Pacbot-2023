from gpiozero import PhaseEnableMotor
from time import sleep

motor = PhaseEnableMotor(19, 26)
motor.forward()

sleep(5)

from gpiozero import PhaseEnableMotor, RotaryEncoder
import Encoder
from time import sleep

motor1 = PhaseEnableMotor(22, 27)
motor2 = PhaseEnableMotor(6, 5)
# rotar1 = RotaryEncoder(23, 24, max_steps=0, bounce_time=300.0)
# rotar2 = RotaryEncoder(17, 25, max_steps=0, bounce_time=300.0)
enc_1 = Encoder.Encoder(23, 24)
enc_2 = Encoder.Encoder(17, 25)
count = 0
motor1.forward(0.5)
motor2.forward(0.5)
while(count < 500):
	print(enc_1.read(), enc_2.read())
	sleep(0.1)
	count+=1

#rotar1.wait_for_rotate()
#print("yeet")
#speed = 0.5
#motor1.backward(speed)
#motor2.backward(speed)
#counter = 0
#while counter < 1000:
#    print(rotar1.steps, rotar2.steps)
#    if counter == 500:
#        motor1.stop()
#        motor2.stop()
#        sleep(0.01)
#        print("turning around")
#        motor1.forward(speed)
#        motor2.forward(speed)
#    # motor1.backward(0.5)
#    # motor2.backward(0.5)
#    counter += 1
#    sleep(0.01)


motor1.stop()
motor2.stop()

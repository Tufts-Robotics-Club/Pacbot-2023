from simple_pid import PID
import time
target = 1.25
pid = PID(1.0, 0.1, 0.05, setpoint=target)
pid.sample_time = 0.1

v = 0

while abs(v-target) > 0.1:
    control = pid(v)
    v += control / 7.5
    print("Going...", v, control)
    time.sleep(0.01)

print("Done!", v)

# NAME: imu_calibration.py
# PURPOSE: calibrating BN055 IMU; will hopefully only ever have to be used once
# AUTHOR: Emma Bethel

import time
import board
import adafruit_bno055
import adafruit_tca9548a


i2c = board.I2C()  # uses board.SCL and board.SDA

# multiplexer
tca = adafruit_tca9548a.TCA9548A(i2c)

# imu (attached to port 0 of multiplexer)
sensor = adafruit_bno055.BNO055_I2C(tca[0])

sensor.offsets_gyroscope = (-1, 129, 0)
sensor.offsets_accelerometer = (3, -52, -8)
sensor.offsets_magnetometer = (54, 773, 286)

TARGET_CALIBRATION_VAL = 3

done = False
while True:
    calibration_status = sensor.calibration_status
    print("Calibration Status (system, gyro, accel, mag): {}".format(calibration_status), "Euler Angle: {}".format(sensor.euler))

    if calibration_status[1] == TARGET_CALIBRATION_VAL and calibration_status[1] == TARGET_CALIBRATION_VAL and calibration_status[2] == TARGET_CALIBRATION_VAL and calibration_status[3] == TARGET_CALIBRATION_VAL:
        print("Target reached!")
        # break

    time.sleep(0.01)



print("final calibration vals (gyro, accel, mag)", sensor.offsets_gyroscope, sensor.offsets_accelerometer, sensor.offsets_magnetometer)

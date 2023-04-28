# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_bno055
import adafruit_tca9548a


i2c = board.I2C()  # uses board.SCL and board.SDA

# multiplexer
tca = adafruit_tca9548a.TCA9548A(i2c)

# imu (attached to port 0 of multiplexer)
sensor = adafruit_bno055.BNO055_I2C(tca[1])

last_val = 0xFFFF


def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result


while True:
    print("Temperature: {} degrees C".format(sensor.temperature))
    """
    print(
        "Temperature: {} degrees C".format(temperature())
    )  # Uncomment if using a Raspberry Pi
    """
    print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    print("Magnetometer (microteslas): {}".format(sensor.magnetic))
    print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    print("Euler angle: {}".format(sensor.euler))
    print("Quaternion: {}".format(sensor.quaternion))
    print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    print("Gravity (m/s^2): {}".format(sensor.gravity))
    print()

    time.sleep(0.01)
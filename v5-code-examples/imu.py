"""
Routine that configures on-board inertial measurement unit (IMU).
"""
from lib.pycubed import cubesat
import time


while True:

    # Read acceleration, magnetometer, gyroscope, temperature.
    accel_x, accel_y, accel_z = cubesat.acceleration
    mag_x, mag_y, mag_z       = cubesat.magnetic
    gyro_x, gyro_y, gyro_z    = cubesat.gyro
    temp = cubesat.temperature

    # Print values.
    print(f"Acc  (m/s^2):   x: {accel_x}\ty: {accel_y}\tz: {accel_z}")
    print(f"Mag     (uT):   x: {mag_x}\ty: {mag_y}\tz: {mag_z}")
    print(f"Gyro (deg/sec): x: {gyro_x}\ty: {gyro_y}\tz: {gyro_z}")
    print(f"Temperature: {temp}C\n")

    # Delay for 1 second
    time.sleep(1)
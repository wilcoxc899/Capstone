# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_mpu6050

i2c = board.I2C()  # uses board.SCL and board.SDA
mpu = adafruit_mpu6050.MPU6050(i2c)

while time() < 15:
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (mpu.acceleration))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f rad/s" % (mpu.gyro))
    print("Temperature: %.2f C" % mpu.temperature)
    print("")
    time.sleep(0.1)
file_path = "imu_data.txt"
with open(file_path, "w") as file:
    try:
        while time() < 15:
            # Read data from the IMU sensor
            accel_data = sensor.get_accel_data()
            gyro_data = sensor.get_gyro_data()

            # Format the data as a string
            data_str = f"Accelerometer: {accel_data}, Gyroscope: {gyro_data}"

            # Write data to the file
            file.write(data_str + "\n")

            # Print data to the console (optional)
            print(data_str)

            # Pause for a short interval (adjust as needed)
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Handle Ctrl+C to exit the loop gracefully
        print("Data logging stopped.")

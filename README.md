# Rocket Control System

This repository contains the source code for a rocket control system that uses an Arduino board with various sensors and actuators. The system is designed to control a model rocket's flight parameters, log sensor data, and communicate with a ground station over LoRa.

The provided code is designed for a rocket or high-altitude vehicle. It integrates various sensors and modules to monitor and control the rocket's flight, log sensor data, transmit it via LoRa communication, and respond to specific conditions during the flight.
to know more about the control system and logic
https://github.com/nafis-exe/AERD_ALT_FC_V1/blob/main/Rocket%20Flight%20Control%20Code%20Explanation.md#rocket-flight-control-code-explanation

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Calibration](#calibration)
- [Contributing](#contributing)
- [License](#license)

## Features

- **IMU Integration**: Utilizes an MPU6050 sensor for accurate Inertial Measurement Unit (IMU) data.
- **GPS**: Integrates with a GPS module for location and satellite data.
- **PID Control**: Implements PID control for pitch, roll, and yaw stabilization.
- **LoRa Communication**: Sends sensor data and rocket status to a ground station via LoRa.
- **Parachute Deployment**: Deploys a parachute at a specified altitude based on sensor data.
- **Data Logging**: Logs sensor data to an SD card for post-flight analysis.

## Hardware Requirements

To run this code, you'll need the following hardware components:

- Arduino board (e.g., mega or stm32)
- MPU6050 accelerometer and gyroscope module
- BMP180 or BMP085 barometric pressure sensor
- GPS module (e.g., NEO-6M)
- Servo motors for pitch, yaw, and roll control
- Parachute deployment servo motor
- SD card module for data logging
- LoRa module (e.g., SX1278 or compatible)
- Buzzer and LED for status indication
MPU6050 Accelerometer/Gyroscope Module:

Connect the VCC pin to 5V on the Arduino.
Connect the GND pin to GND on the Arduino.
Connect the SDA pin to A4 (analog pin) on the Arduino.
Connect the SCL pin to A5 (analog pin) on the Arduino.
BMP180/BMP085 Barometric Pressure Sensor:

Connect the VCC pin to 5V on the Arduino.
Connect the GND pin to GND on the Arduino.
Connect the SDA pin to A4 (analog pin) on the Arduino.
Connect the SCL pin to A5 (analog pin) on the Arduino.
GPS Module (e.g., NEO-6M):

Connect the VCC pin to 5V on the Arduino.
Connect the GND pin to GND on the Arduino.
Connect the RX pin of the GPS module to TX (pin 1) on the Arduino (you may need a voltage divider or level shifter).
Connect the TX pin of the GPS module to RX (pin 0) on the Arduino (you may need a voltage divider or level shifter).
Servo Motors (Pitch, Yaw, Roll, and Parachute):

Connect the signal (control) wire of each servo to the corresponding PWM pins on the Arduino (e.g., pitchServo to pin 3, yawServo to pin 4, rollServo1 to pin 5, rollServo2 to pin 6, and parachuteServo to pin 7).
Buzzer and LED:

Connect one leg of the buzzer to a digital pin (e.g., pin 10).
Connect one leg of the LED to a digital pin (e.g., pin 11).
LoRa Module (e.g., SX1278):

Connect the LoRa module according to its pinout to the appropriate digital pins on the Arduino (e.g., SPI pins for communication).
SD Card Module (for Data Logging):

Connect the SD card module according to its pinout to the appropriate digital pins on the Arduino (e.g., MOSI, MISO, SCK, CS).

## Software Requirements

You'll need the following software:

- Arduino IDE (https://www.arduino.cc/en/software)
- Required Arduino libraries (Adafruit MPU6050, Adafruit BMP085, Adafruit GPS, PID_v1, TinyGPS++, SoftwareSerial, and LoRa)
- will add a processing flight telemetrty visulizer soon
## Installation

1. Clone this repository to your local machine:

   ```shell
   git clone https://github.com/your-username/rocket-control-system.git
   Open the Arduino IDE and load the rocket_control_system.ino sketch.

Install the required libraries using the Arduino Library Manager:

- Adafruit MPU6050
- Adafruit BMP085 (or BMP180)
- Adafruit GPS
- PID_v1
- TinyGPS++
- SoftwareSerial
- LoRa
- Configure the Arduino IDE for your specific board(download stm32 manager for stm32 board)

# Usage
Before launching the rocket, ensure that you have calibrated the MPU6050 sensor by following the calibration procedure described below.

# Calibration:

To calibrate the MPU6050 sensor, you can send a calibration command to the Arduino board either via serial communication or LoRa. Make sure the rocket is on a flat, stable surface and not moving during calibration.

shell
Copy code
Send calibration command over serial (replace COMx with your serial port)
echo "CALIBRATE" > COMx
Launch:

After calibration, you can initiate the rocket launch. Once the rocket reaches the target altitude and conditions are met, the system will automatically deploy the parachute. The status and sensor data will be transmitted via LoRa to a ground station.

# Monitoring:

You can monitor the rocket's status and data from the ground station. The transmitted data includes pitch, roll, yaw angles, altitude, latitude, and longitude.

# Calibration
Calibrating the MPU6050 sensor is a crucial step to ensure accurate measurements. During calibration, the sensor should be placed on a flat and stable surface, and the calibration command should be sent.

Ensure the rocket is on a stable surface.
Send the calibration command to the Arduino (as shown in the Usage section).
Wait for the calibration process to complete. The offsets will be printed to the Serial Monitor.
# Contributing
Contributions are welcome! If you'd like to contribute to this project, please follow these steps:
for now i would love my teamates to contribute 

Fork the repository.
Create a new branch for your feature or bug fix.
Make your changes.
Test your changes thoroughly.
Create a pull request with a clear description of your changes.
# License
This project is licensed under the MIT License. See the LICENSE file for details.






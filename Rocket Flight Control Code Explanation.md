# Rocket Flight Control Code Explanation

## Purpose
The provided code is designed for a rocket or high-altitude vehicle. It integrates various sensors and modules to monitor and control the rocket's flight, log sensor data, transmit it via LoRa communication, and respond to specific conditions during the flight.

## Key Components and Their Roles

1. **MPU6050 Accelerometer/Gyroscope Module:**
   - Measures rocket orientation (pitch, roll, yaw) and accelerations.
   - Uses a Kalman filter to refine sensor data and improve accuracy.
   - PID controllers adjust servo motor positions based on orientation data.

2. **BMP180/BMP085 Barometric Pressure Sensor:**
   - Measures altitude based on atmospheric pressure.
   - Provides initial altitude reading for reference.

3. **GPS Module (NEO-6M):**
   - Receives GPS data (latitude, longitude, satellites).
   - Indicates GPS status using an LED and buzzer.

4. **Servo Motors (Pitch, Yaw, Roll, Parachute):**
   - Adjust rocket orientation using PID controllers.
   - Control parachute deployment servo.

5. **Buzzer and LED:**
   - Provide audio and visual feedback during calibration and GPS status.

6. **LoRa Module (SX1278):**
   - Transmit sensor data (including pitch, roll, yaw, altitude) to a ground station.
   - Receive commands for calibration and pre-launch checks.

7. **SD Card Module (Data Logging):**
   - Logs sensor data to an SD card for later analysis.

## Code Logic and Flow

### Setup Phase (setup())
- Initialize communication with sensors, GPS, and modules.
- Calibrate the MPU6050 accelerometer and gyroscope (optional).
- Configure PID controllers for servo motor control.
- Initialize servo motors, LEDs, and the buzzer.
- Open an SD card file for data logging.

### Main Loop (loop())
- Read sensor data from the MPU6050 and BMP180.
- Use the Kalman filter to improve orientation data (pitch, roll, yaw).
- Apply PID control to adjust servo motor positions based on orientation.
- Log sensor data to the SD card and transmit it via LoRa.
- Check for parachute deployment conditions:
  - Altitude reaches a predefined target.
  - A set time delay (deployDelay) after launch has passed.
  - No abnormal IMU (Inertial Measurement Unit) conditions.
- Update and check GPS status, providing feedback using an LED and buzzer.
- Check for calibration and pre-launch commands via serial or LoRa communication.

### Calibration (startCalibration())
- Triggered by a command (serial or LoRa) to initiate MPU6050 calibration.
- Instructs the user to place the MPU6050 on a flat, stable surface.
- Collects accelerometer data for calibration and stores offsets.

### Data Transmission (transmitSensorData())
- Converts sensor data into a JSON format.
- Transmits sensor data via LoRa communication.
- Displays sensor data on the serial monitor for debugging.

### Parachute Deployment (checkParachuteDeployment())
- Monitors altitude change and acceleration after a certain time post-launch.
- Detects abnormal IMU conditions (abrupt changes).
- Deploys the parachute servo when conditions are met.

### GPS Status (updateGPS())
- Reads and decodes GPS data.
- Checks the number of satellites for GPS status.
- Provides visual and audio feedback based on GPS status.

## How to Interact with the Code
- To calibrate the MPU6050, send a calibration command via serial or LoRa.
- To trigger pre-launch checks and initiate data transmission, send a pre-launch command via serial or LoRa.
- The code will continuously monitor altitude and acceleration for parachute deployment conditions.
- Feedback regarding GPS status (good or poor) will be provided through an LED and buzzer.
- Sensor data is logged to an SD card and transmitted via LoRa for monitoring.

Please note that specific commands and data formats for serial and LoRa communication may need to be defined based on your ground station's requirements. Additionally, configure LoRa settings (e.g., frequency, spreading factor) as needed for reliable communication.

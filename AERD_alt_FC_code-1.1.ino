#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include <PID_v1.h>
#include <SPI.h>
#include <SD.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

Servo pitchServo;
Servo yawServo;
Servo rollServo1;
Servo rollServo2;
Servo parachuteServo;

const int pitchServoPin = 3;
const int yawServoPin = 4;
const int rollServo1Pin = 5;
const int rollServo2Pin = 6;
const int parachuteServoPin = 7;
const int buzzerPin = 10; // Change to your buzzer pin
const int ledPin = 11;    // Change to your LED pin

double pidInputPitch, pidInputRoll, pidInputYaw;
double pidOutputPitch, pidOutputRoll, pidOutputYaw;
double pidSetpointPitch = 0.0;
double pidSetpointRoll = 0.0;
double pidSetpointYaw = 0.0;

double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;

PID pitchPID(&pidInputPitch, &pidOutputPitch, &pidSetpointPitch, Kp, Ki, Kd, DIRECT);
PID rollPID(&pidInputRoll, &pidOutputRoll, &pidSetpointRoll, Kp, Ki, Kd, DIRECT);
PID yawPID(&pidInputYaw, &pidOutputYaw, &pidSetpointYaw, Kp, Ki, Kd, DIRECT);

const unsigned long sampleTime = 10;
unsigned long previousMillis = 0;

double pitchAngle = 0.0;
double rollAngle = 0.0;
double yawAngle = 0.0;

const double targetAltitude = 1000.0;
bool parachuteDeployed = false;
unsigned long launchTime = 0;
const unsigned long deployDelay = 10000;

bool imuAbnormal = false;
double previousAcceleration = 0.0;
unsigned long imuCheckStartTime = 0;
const unsigned long imuCheckInterval = 10000;

double initialAltitude = 0.0;
double maxAltitudeChange = 0.0;
double maxAccelerationChange = 0.0;

// Kalman filter parameters
double Q_angle = 0.001;
double Q_bias = 0.003;
double R_measure = 0.03;

double gyroBiasX = 0.0;
double gyroBiasY = 0.0;
double gyroBiasZ = 0.0;

File dataFile; // File object for SD card

// GPS Setup
TinyGPSPlus gps;
SoftwareSerial gpsSerial(8, 9); // RX, TX

bool isCalibrating = false; // Flag to indicate whether calibration is in progress

// LoRa Configuration
const int loraCsPin = 8;
const int loraResetPin = 9;
const int loraIntPin = 7;
const long loraFrequency = 433E6; // Set your LoRa frequency here
const int loraSignalThreshold = -80; // Adjust signal threshold as needed
bool isReadyToLaunch = false;
bool isLaunched = false;

void kalmanFilterUpdate(double &angle, double &bias, double rate, double measurement) {
  double P = 0.0;
  double K = 0.0;

  // Prediction
  angle += (rate - bias) * sampleTime;

  P += Q_angle * sampleTime;
  bias += Q_bias * sampleTime;

  // Measurement update
  K = P / (P + R_measure);
  angle += K * (measurement - angle);
  P *= (1.0 - K);
}

void startCalibration() {
  Serial.println("Starting MPU6050 Calibration...");
  Serial.println("Place the MPU6050 on a flat, stable surface for calibration.");
  Serial.println("Ensure that the device is not moving during calibration.");

  // Set the flag to indicate that calibration is in progress
  isCalibrating = true;

  // Call the calibration function
  calibrateMPU6050();

  // Calibration complete
  Serial.println("MPU6050 calibration complete.");
  Serial.println("Calibration offsets stored.");

  // Reset the flag
  isCalibrating = false;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  // Buzzer and LED setup
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // Indicate IMU calibration
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(ledPin, HIGH);
  delay(3000); // Wait for 3 seconds to indicate calibration
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050. Please check wiring.");
    while (1);
  }

  if (!bmp.begin()) {
    Serial.println("Failed to find BMP180. Please check wiring.");
    while (1);
  }

  // GPS Initialization
  gpsSerial.begin(9600);

  pitchServo.attach(pitchServoPin);
  yawServo.attach(yawServoPin);
  rollServo1.attach(rollServo1Pin);
  rollServo2.attach(rollServo2Pin);
  parachuteServo.attach(parachuteServoPin);

  pitchPID.SetOutputLimits(-90.0, 90.0);
  rollPID.SetOutputLimits(-90.0, 90.0);
  yawPID.SetOutputLimits(-90.0, 90.0);

  pitchPID.SetSampleTime(sampleTime);
  rollPID.SetSampleTime(sampleTime);
  yawPID.SetSampleTime(sampleTime);

  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);

  pitchServo.write(90);
  yawServo.write(90);
  rollServo1.write(90);
  rollServo2.write(90);

  launchTime = millis();
  imuCheckStartTime = millis();
  initialAltitude = bmp.readAltitude();

  // Create a file on the SD card
  dataFile = SD.open("sensor_data.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println("Time (ms), Pitch (deg), Roll (deg), Yaw (deg), Altitude (m), Latitude, Longitude");
    dataFile.close();
  } else {
    Serial.println("Error opening data file.");
  }

  // LoRa Setup
  LoRa.setPins(loraCsPin, loraResetPin, loraIntPin);
  if (!LoRa.begin(loraFrequency)) {
    Serial.println("LoRa initialization failed. Check your wiring.");
    while (1);
  }

  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(12);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  Serial.println("LoRa initialized.");

  // Calibrate the MPU6050
  calibrateMPU6050();
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long timeSinceLastUpdate = currentMillis - previousMillis;

  if (timeSinceLastUpdate >= sampleTime) {
    previousMillis = currentMillis;

    sensors_event_t accelEvent, gyroEvent, tempEvent;
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

    if (!isCalibrating) {
      double accelPitch = atan2(-accelEvent.acceleration.x, sqrt(accelEvent.acceleration.y * accelEvent.acceleration.y + accelEvent.acceleration.z * accelEvent.acceleration.z)) * RAD_TO_DEG;
      double gyroRoll = gyroEvent.gyro.x * 0.01;
      double gyroYaw = gyroEvent.gyro.z * 0.01;

      // Update Kalman filters
      kalmanFilterUpdate(pitchAngle, gyroBiasX, gyroEvent.gyro.y, accelPitch);
      kalmanFilterUpdate(rollAngle, gyroBiasY, gyroRoll, accelEvent.acceleration.x);
      kalmanFilterUpdate(yawAngle, gyroBiasZ, gyroYaw, accelEvent.acceleration.z);

      pidSetpointPitch = 0.0;
      pidSetpointRoll = 0.0;
      pidSetpointYaw = 0.0;

      pidInputPitch = pitchAngle;
      pidInputRoll = rollAngle;
      pidInputYaw = yawAngle;

      pitchPID.Compute();
      rollPID.Compute();
      yawPID.Compute();

      pitchServo.write(90 + pidOutputPitch);
      rollServo1.write(90 + pidOutputRoll);
      rollServo2.write(90 - pidOutputRoll);
      yawServo.write(90 + pidOutputYaw);

      logSensorData(); // Log sensor data to SD card
      transmitSensorData(); // Transmit sensor data via LoRa
    }
  }

  checkParachuteDeployment(); // Moved this function call outside of the if condition
  updateGPS(); // Read and update GPS data

  // Check for incoming serial data
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'C' || command == 'c') {
      // Start calibration
      startCalibration();
    } else if (command == 'P' || command == 'p') {
      // Send pre-launch feedback
      sendPreLaunchFeedback();
    }
  }

  // Check for incoming LoRa data
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }
    if (receivedData == "CALIBRATE") {
      // Start calibration
      startCalibration();
    } else if (receivedData == "PRE_LAUNCH") {
      // Send pre-launch feedback
      sendPreLaunchFeedback();
    }
  }
}

void logSensorData() {
  // Open the data file in append mode
  dataFile = SD.open("sensor_data.txt", FILE_WRITE);

  if (dataFile) {
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

    // Get current altitude
    double currentAltitude = bmp.readAltitude();

    // Log data to the SD card
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.print(pidInputPitch);
    dataFile.print(",");
    dataFile.print(pidInputRoll);
    dataFile.print(",");
    dataFile.print(pidInputYaw);
    dataFile.print(",");
    dataFile.print(currentAltitude);
    dataFile.print(",");
    dataFile.print(gps.location.lat(), 6);
    dataFile.print(",");
    dataFile.print(gps.location.lng(), 6);
    dataFile.println();

    // Close the file
    dataFile.close();
  } else {
    Serial.println("Error opening data file.");
  }
}

void transmitSensorData() {
  String sensorData;

  sensors_event_t accelEvent, gyroEvent, tempEvent;
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  // Get current altitude
  double currentAltitude = bmp.readAltitude();

  // Create a JSON formatted string with sensor data
  sensorData = "{\"Time\": " + String(millis()) +
               ", \"Pitch\": " + String(pidInputPitch) +
               ", \"Roll\": " + String(pidInputRoll) +
               ", \"Yaw\": " + String(pidInputYaw) +
               ", \"Altitude\": " + String(currentAltitude) +
               ", \"Latitude\": " + String(gps.location.lat(), 6) +
               ", \"Longitude\": " + String(gps.location.lng(), 6) + "}";

  // Convert the string to a char array
  char charBuffer[sensorData.length() + 1];
  sensorData.toCharArray(charBuffer, sizeof(charBuffer));

  // Send the data via LoRa
  LoRa.beginPacket();
  LoRa.print(charBuffer);
  LoRa.endPacket();

  // Print data to Serial
  Serial.println(sensorData);
}

void checkParachuteDeployment() {
  unsigned long currentMillis = millis();
  sensors_event_t accelEvent;

  double currentAltitude = bmp.readAltitude();
  double altitudeChange = currentAltitude - initialAltitude;

  if (altitudeChange > maxAltitudeChange) {
    maxAltitudeChange = altitudeChange;
  }

  mpu.getEvent(&accelEvent, nullptr, nullptr); // Get accelerometer data

  double accelerationChange = abs(accelEvent.acceleration.z - previousAcceleration);

  if (accelerationChange > maxAccelerationChange) {
    maxAccelerationChange = accelerationChange;
  }

  if (currentMillis - launchTime >= 10000 && maxAltitudeChange >= 50.0 && maxAccelerationChange >= 5.0) {
    imuAbnormal = true;
  }

  if (!parachuteDeployed && currentAltitude >= targetAltitude && currentMillis - launchTime >= deployDelay && !imuAbnormal) {
    parachuteServo.write(90);
    parachuteDeployed = true;
  }

  previousAcceleration = accelEvent.acceleration.z;
}

void updateGPS() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      int satellites = gps.satellites.value();

      // Check GPS status and indicate with LED and buzzer
      if (satellites >= 4) {
        digitalWrite(ledPin, HIGH); // GPS status is good
        noTone(buzzerPin);
        Serial.println("GPS Status: Good");
      } else {
        digitalWrite(ledPin, LOW); // GPS status is poor
        tone(buzzerPin, 1000); // Beep the buzzer to indicate poor GPS status
        Serial.println("GPS Status: Poor");
      }
    }
  }
}

void calibrateMPU6050() {
  const int numSamples = 1000; // Number of samples to collect for calibration

  // Variables to store offsets
  double accelXOffset = 0.0;
  double accelYOffset = 0.0;
  double accelZOffset = 0.0;

  // Read the current sensitivity range
  uint8_t accelRange = mpu.getAccelerometerRange();

  // Set the desired sensitivity range for calibration (adjust as needed)
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  Serial.println("Place the MPU6050 on a flat, stable surface for calibration.");
  Serial.println("Ensure that the device is not moving during calibration.");
  delay(5000); // Wait for 5 seconds to prepare

  // Collect and average sensor data for calibration
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t accelEvent;
    mpu.getEvent(&accelEvent, nullptr, nullptr);

    accelXOffset += accelEvent.acceleration.x;
    accelYOffset += accelEvent.acceleration.y;
    accelZOffset += accelEvent.acceleration.z;

    delay(5); // Sampling interval
  }

  // Calculate average offsets
  accelXOffset /= numSamples;
  accelYOffset /= numSamples;
  accelZOffset /= numSamples;

  // Restore the original sensitivity range
  mpu.setAccelerometerRange(accelRange);

  // Store the calibration offsets in separate variables
  double accelXCalibrationOffset = accelXOffset;
  double accelYCalibrationOffset = accelYOffset;
  double accelZCalibrationOffset = accelZOffset;

  Serial.println("MPU6050 calibration complete.");
  Serial.print("Accelerometer offsets (mg): ");
  Serial.print(accelXCalibrationOffset);
  Serial.print(", ");
  Serial.print(accelYCalibrationOffset);
  Serial.print(", ");
  Serial.print(accelZCalibrationOffset);
  Serial.println();
}

void sendPreLaunchFeedback() {
  unsigned long currentMillis = millis();
  double currentAltitude = bmp.readAltitude();
  int rssi = LoRa.packetRssi();

  if (!isReadyToLaunch) {
    if (currentAltitude >= targetAltitude) {
      isReadyToLaunch = true;
      Serial.println("Ready to launch.");
      Serial.print("RSSI: ");
      Serial.println(rssi);

      // Send feedback via LoRa
      String feedback = "READY_TO_LAUNCH RSSI: " + String(rssi);
      LoRa.beginPacket();
      LoRa.print(feedback);
      LoRa.endPacket();
    }
  } else {
    if (currentMillis - launchTime >= 10000) {
      Serial.println("In flight.");

      // Send feedback via LoRa
      String feedback = "IN_FLIGHT RSSI: " + String(rssi);
      LoRa.beginPacket();
      LoRa.print(feedback);
      LoRa.endPacket();
    }
  }
}


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

Adafruit_MPU6050 mpu;               // Create an instance of the MPU6050 sensor
Adafruit_BMP085 bmp;                // Create an instance of the BMP180 sensor

Servo pitchServo;                   // Servo for pitch control
Servo yawServo;                     // Servo for yaw control
Servo rollServo1;                   // Servo for roll control (motor 1)
Servo rollServo2;                   // Servo for roll control (motor 2)
Servo parachuteServo;               // Servo for parachute deployment

const int pitchServoPin = 3;        // Pin for pitch servo
const int yawServoPin = 4;          // Pin for yaw servo
const int rollServo1Pin = 5;        // Pin for roll servo (motor 1)
const int rollServo2Pin = 6;        // Pin for roll servo (motor 2)
const int parachuteServoPin = 7;    // Pin for parachute servo
const int buzzerPin = 10;           // Buzzer pin
const int ledPin = 11;              // LED pin

double pidInputPitch, pidInputRoll, pidInputYaw;  // PID input variables
double pidOutputPitch, pidOutputRoll, pidOutputYaw;  // PID output variables
double pidSetpointPitch = 0.0;      // Pitch PID setpoint
double pidSetpointRoll = 0.0;       // Roll PID setpoint
double pidSetpointYaw = 0.0;        // Yaw PID setpoint

double Kp = 1.0;                    // PID proportional constant
double Ki = 0.0;                    // PID integral constant
double Kd = 0.0;                    // PID derivative constant

PID pitchPID(&pidInputPitch, &pidOutputPitch, &pidSetpointPitch, Kp, Ki, Kd, DIRECT);
PID rollPID(&pidInputRoll, &pidOutputRoll, &pidSetpointRoll, Kp, Ki, Kd, DIRECT);
PID yawPID(&pidInputYaw, &pidOutputYaw, &pidSetpointYaw, Kp, Ki, Kd, DIRECT);

const unsigned long sampleTime = 10;  // PID sample time
unsigned long previousMillis = 0;     // Time of the previous PID update

double pitchAngle = 0.0;              // Current pitch angle
double rollAngle = 0.0;               // Current roll angle
double yawAngle = 0.0;                // Current yaw angle

const double targetAltitude = 1000.0;  // Target altitude for parachute deployment
bool parachuteDeployed = false;        // Flag to indicate if the parachute has been deployed
unsigned long launchTime = 0;          // Time of launch
const unsigned long deployDelay = 10000;  // Delay after launch before deploying parachute

bool imuAbnormal = false;              // Flag for abnormal IMU behavior
double previousAcceleration = 0.0;     // Previous acceleration reading
unsigned long imuCheckStartTime = 0;   // Time to start checking for abnormal IMU behavior
const unsigned long imuCheckInterval = 10000;  // Interval for checking IMU behavior

double initialAltitude = 0.0;         // Initial altitude for altitude change calculation
double maxAltitudeChange = 0.0;       // Maximum altitude change
double maxAccelerationChange = 0.0;   // Maximum acceleration change

// Kalman filter parameters
double Q_angle = 0.001;
double Q_bias = 0.003;
double R_measure = 0.03;

double gyroBiasX = 0.0;
double gyroBiasY = 0.0;
double gyroBiasZ = 0.0;

File dataFile;  // File object for SD card

// GPS Setup
TinyGPSPlus gps;                        // Create an instance of the TinyGPS++ object
SoftwareSerial gpsSerial(8, 9);         // Create a software serial connection for GPS

bool isCalibrating = false;              // Flag to indicate whether calibration is in progress

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

#include <Wire.h>
#include <MPU6050.h>
#include "KalmanFilter.h"

MPU6050 mpu;
KalmanFilter kalmanX;
KalmanFilter kalmanY;

// Motor driver pins
#define ENA 17
#define IN1 16
#define IN2 15
#define ENB 19
#define IN3 5
#define IN4 23

// PID constants
float Kp = 5 0.3;
float Ki = 0;
float Kd = 0;

// PID variables
float setpoint = 0.0;
float input, output;
float previousError = 0.0;
float integral = 0.0;

// Timer
unsigned long timer;
float dt;

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }

  // Initialize motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  mpu.setXAccelOffset(-2911);
  mpu.setYAccelOffset(-1996);
  mpu.setZAccelOffset(5649);
  mpu.setXGyroOffset(145);
  mpu.setYGyroOffset(-24);
  mpu.setZGyroOffset(24);
  /* mpu.setXAccelOffset(-2788);
  mpu.setYAccelOffset(807);
  mpu.setZAccelOffset(1210);
  mpu.setXGyroOffset(66);
  mpu.setYGyroOffset(-12);
  mpu.setZGyroOffset(16);*/

  timer = micros();
}

void loop() {
  // Calculate delta time
  dt = (float)(micros() - timer) / 1000000;
  timer = micros();

  // Read angle from MPU6050
  input = readAngle(dt);

  // Compute PID output
  float error = setpoint - input;
  integral += error * dt;
  float derivative = (error - previousError) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  // Control motors based on PID output
  setMotorSpeed(output);

  delay(10);  // Small delay for stability
}

float readAngle(float dt) {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert gyroscope values to angles
  float gyroXrate = (float)gx / 131.0;
  float gyroYrate = (float)gy / 131.0;

  // Convert accelerometer values to angles
  float accelXangle = atan2(ay, az) * 180 / PI;
  float accelYangle = atan2(ax, az) * 180 / PI;

  // Apply Kalman filter
  float angleX = kalmanX.getAngle(accelXangle, gyroXrate, dt);
  float angleY = kalmanY.getAngle(accelYangle, gyroYrate, dt);

  return angleY;  // Use the appropriate angle for balancing
}

void setMotorSpeed(float speed) {
  int motorSpeed = constrain(abs(speed), 0, 255);

  if (speed > 0) {
    analogWrite(ENA, motorSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    analogWrite(ENB, motorSpeed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    analogWrite(ENA, motorSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(ENB, motorSpeed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

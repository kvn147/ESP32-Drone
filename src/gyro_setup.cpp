/* Copyright (c) 2025
 * Interfacing with a MPU6050 gyroscope sensor for ESP32.
 */
#include <Arduino.h>
#include <Wire.h>

const int MPU6050_ADDRESS = 0x68; // I2C address of MPU6050
int16_t gyroX, gyroY, gyroZ; 

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 100000); // sda, scl, clock speed
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU−6050)
  Wire.endTransmission(true);
  Serial.println("Setup complete");
}

void loop() {}
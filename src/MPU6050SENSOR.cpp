/*  
File: MPU6050SENSOR.cpp
Author: Theodoros Taloumtzis
Date: 10/12/2024
Purpose: Implements the MPU6050Sensor class functions.
*/

#include "MPU6050SENSOR.h"
#include <math.h>

// Constructor
MPU6050Sensor::MPU6050Sensor() {
    _ax = 0;
    _ay = 0;
    _az = 0;
    _gx = 0;
    _gy = 0;
    _gz = 0;

    _acc_x = 0;
    _acc_y = 0;
    _acc_z = 0;

    _angle_x = 0;
    _angle_y = 0;
    _angle_z = 0;

    _acc_error_x = 0;
    _acc_error_y = 0;

    _gyro_error_x = 0;
    _gyro_error_y = 0;
    _gyro_error_z = 0;

    _previous_time = 0;
}

// Destructor
MPU6050Sensor::~MPU6050Sensor() { }

void MPU6050Sensor::setup() {
    // Initialize I2C connection
    Wire.begin();

    // Check if the sensor connection is successful
    if (!_mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1); // Enter an infinite loop to prevent code execution
    }

    // Initialize the MPU6050 sensor
    _mpu.initialize();
    _mpu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

    calculateIMUErrors();
}

void MPU6050Sensor::loop() {
    // Read the raw accelerometer and gyroscope data
    _mpu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

    // Convert raw accelerometer readings to g-force values
    _acc_x = _ax / 16384.0;
    _acc_y = _ay / 16384.0;
    _acc_z = _az / 16384.0;

    // Calculate the angles using accelerometer data
    _angle_x = atan2(_acc_y, _acc_z) * 180 / PI;  // Roll angle in degrees
    _angle_y = atan2(-_acc_x, sqrt(_acc_y * _acc_y + _acc_z * _acc_z)) * 180 / PI;  // Pitch angle in degrees

    // Calculate the approximate yaw angle using gyro data and time delta
    unsigned long current_time = millis();
    float elapsed_time = (current_time - _previous_time) / 1000.0; // Convert to seconds
    _previous_time = current_time;

    float gyro_z = _gz / 131.0; // Convert gyro Z to degrees/second
    _angle_z += gyro_z * elapsed_time; // Integrate for yaw angle (_angle_z)

    // Print the sensor values
    Serial.print("AccX: ");
    Serial.println(_acc_x);
    Serial.print("AccY: ");
    Serial.println(_acc_y);
    Serial.print("AccZ: ");
    Serial.println(_acc_z);
    Serial.print("AngleX: ");
    Serial.println(_angle_x);
    Serial.print("AngleY: ");
    Serial.println(_angle_y);
    Serial.print("AngleZ: ");
    Serial.println(_angle_z);
}

// Getters
float MPU6050Sensor::getAccX() {
    return _acc_x;
}
float MPU6050Sensor::getAccY() {
    return _acc_y;
}
float MPU6050Sensor::getAccZ() {
    return _acc_z;
}

float MPU6050Sensor::getAngleX() {
    return _angle_x;
}
float MPU6050Sensor::getAngleY() {
    return _angle_y;
}
float MPU6050Sensor::getAngleZ() {
    return _angle_z;
}

void MPU6050Sensor::calculateIMUErrors() {
    int sample_count = 256; // Number of samples to average
    for (int i = 0; i < sample_count; i++) {
        // Read raw accelerometer and gyroscope data
        _mpu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

        // Calculate accelerometer angles and sum them for averaging
        _acc_x = _ax / 16384.0;
        _acc_y = _ay / 16384.0;
        _acc_z = _az / 16384.0;

        _acc_error_x += atan2(_acc_y, sqrt(_acc_x * _acc_x + _acc_z * _acc_z)) * 180 / PI;
        _acc_error_y += atan2(-_acc_x, sqrt(_acc_y * _acc_y + _acc_z * _acc_z)) * 180 / PI;

        // Sum gyroscope readings for averaging
        _gyro_error_x += _gx / 131.0;
        _gyro_error_y += _gy / 131.0;
        _gyro_error_z += _gz / 131.0;
    }

    // Average the error values
    _acc_error_x /= sample_count;
    _acc_error_y /= sample_count;
    _gyro_error_x /= sample_count;
    _gyro_error_y /= sample_count;
    _gyro_error_z /= sample_count;

    // Print the error values to the Serial Monitor for reference
    Serial.print("AccErrorX: ");
    Serial.println(_acc_error_x);
    Serial.print("AccErrorY: ");
    Serial.println(_acc_error_y);
    Serial.print("GyroErrorX: ");
    Serial.println(_gyro_error_x);
    Serial.print("GyroErrorY: ");
    Serial.println(_gyro_error_y);
    Serial.print("GyroErrorZ: ");
    Serial.println(_gyro_error_z);
}

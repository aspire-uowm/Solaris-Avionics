/*  
File: ICM20948SENSOR.cpp
Author: Theodoros Taloumtzis
Date: 10/02/2025
Purpose: Implements the ICM20948Sensor class with quaternion calculations, absolute yaw, and calibration.
*/

#include "ICM20948SENSOR.h"

Madgwick filter;

ICM20948Sensor::ICM20948Sensor(TwoWire* wire) : _wire(wire), _icm(*wire, 0x69) {
    _ax = _ay = _az = 0.0;
    _gx = _gy = _gz = 0.0;
    _mx = _my = _mz = 0.0;
    _acc_x = _acc_y = _acc_z = 0.0;
    _angle_x = _angle_y = _angle_z = 0.0;
    _velocity_x = _velocity_y = _velocity_z = 0.0;
    _previous_time = micros();
    _q0 = 1.0; _q1 = _q2 = _q3 = 0.0;
}

ICM20948Sensor::~ICM20948Sensor() {}

void ICM20948Sensor::setup() {
    Serial.begin(115200);
    _wire->begin();
    
    if (_icm.begin() < 0) {
        Serial.println("ICM-20948 connection failed");
        while (1);
    }
    Serial.println("ICM-20948 connected successfully!");
    
    _icm.configMag(); // Enable magnetometer
    calibrateIMU();
    filter.begin(100); // Initialize Madgwick filter with 100Hz update rate
}

void ICM20948Sensor::loop() {
    if (_icm.readSensor() == 1) { 
        _ax = _icm.getAccelX_mss() - _acc_bias_x;
        _ay = _icm.getAccelY_mss() - _acc_bias_y;
        _az = _icm.getAccelZ_mss() - _acc_bias_z;
        _gx = _icm.getGyroX_rads() - _gyro_bias_x;
        _gy = _icm.getGyroY_rads() - _gyro_bias_y;
        _gz = _icm.getGyroZ_rads() - _gyro_bias_z;
        _mx = _icm.getMagX_uT();
        _my = _icm.getMagY_uT();
        _mz = _icm.getMagZ_uT();

        unsigned long current_time = micros();
        float elapsed_time = (current_time - _previous_time) / 1.0e6;
        _previous_time = current_time;

        // Update velocity
        updateVelocity();

        // Update quaternion using Madgwick filter (Now includes magnetometer)
        filter.update(_gx, _gy, _gz, _ax, _ay, _az, _mx, _my, _mz);
        _q0 = filter.q0;
        _q1 = filter.q1;
        _q2 = filter.q2;
        _q3 = filter.q3;
    }
}

void ICM20948Sensor::updateVelocity() {
    float dt = (_previous_time - micros()) / 1.0e6;
    _velocity_x += _ax * dt;
    _velocity_y += _ay * dt;
    _velocity_z += _az * dt;
}

void ICM20948Sensor::resetVelocity() {
    _velocity_x = _velocity_y = _velocity_z = 0.0;
}

void ICM20948Sensor::calibrateIMU() {
    Serial.println("Calibrating IMU, please keep it still...");
    int sample_count = 500;
    _acc_bias_x = _acc_bias_y = _acc_bias_z = 0.0;
    _gyro_bias_x = _gyro_bias_y = _gyro_bias_z = 0.0;
    
    for (int i = 0; i < sample_count; i++) {
        if (_icm.readSensor() == 1) {
            _acc_bias_x += _icm.getAccelX_mss();
            _acc_bias_y += _icm.getAccelY_mss();
            _acc_bias_z += _icm.getAccelZ_mss();
            _gyro_bias_x += _icm.getGyroX_rads();
            _gyro_bias_y += _icm.getGyroY_rads();
            _gyro_bias_z += _icm.getGyroZ_rads();
        }
        delay(10);
    }
    _acc_bias_x /= sample_count;
    _acc_bias_y /= sample_count;
    _acc_bias_z /= sample_count;
    _gyro_bias_x /= sample_count;
    _gyro_bias_y /= sample_count;
    _gyro_bias_z /= sample_count;
    Serial.println("Calibration complete.");
}

void ICM20948Sensor::getEulerAngles(float &roll, float &pitch, float &yaw) {
    roll = atan2(2.0f * (_q0 * _q1 + _q2 * _q3), 1.0f - 2.0f * (_q1 * _q1 + _q2 * _q2)) * 180.0 / PI;
    pitch = asin(2.0f * (_q0 * _q2 - _q3 * _q1)) * 180.0 / PI;
    yaw = atan2(2.0f * (_q0 * _q3 + _q1 * _q2), 1.0f - 2.0f * (_q2 * _q2 + _q3 * _q3)) * 180.0 / PI;

    // Ensure yaw remains in the range [0, 360]
    if (yaw < 0) yaw += 360.0;
}

float ICM20948Sensor::getQuaternionQ0() { return _q0; }
float ICM20948Sensor::getQuaternionQ1() { return _q1; }
float ICM20948Sensor::getQuaternionQ2() { return _q2; }
float ICM20948Sensor::getQuaternionQ3() { return _q3; }

float ICM20948Sensor::getVelocityX() { return _velocity_x; }
float ICM20948Sensor::getVelocityY() { return _velocity_y; }
float ICM20948Sensor::getVelocityZ() { return _velocity_z; }

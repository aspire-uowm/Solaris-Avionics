/*  
File: ICM20948SENSOR.cpp
Author: Theodoros Taloumtzis
Date: 10/02/2025
Purpose: Implements the ICM20948Sensor class with quaternion calculations, absolute yaw, and calibration using Adafruit Madgwick Filter.
*/

#include "ICM20948SENSOR.h"

ICM20948Sensor::ICM20948Sensor(TwoWire* wire) : _wire(wire), _icm(*wire, 0x69) {
    _ax = _ay = _az = 0.0;
    _gx = _gy = _gz = 0.0;
    _mx = _my = _mz = 0.0;
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
    _filter.begin(100); // Initialize Adafruit Madgwick filter with 100Hz update rate
}

void ICM20948Sensor::loop() {
    static unsigned long last_update_time = 0;
    unsigned long current_time = micros();
    float elapsed_time = (current_time - last_update_time) / 1.0e6;
    
    if (elapsed_time >= 0.01) { // Run at ~100Hz
        last_update_time = current_time;

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

            // Update quaternion using Adafruit Madgwick filter
            _filter.update(_gx, _gy, _gz, _ax, _ay, _az, _mx, _my, _mz);
            _filter.getQuaternion(&_q0, &_q1, &_q2, &_q3);
        }   
    }
}

void ICM20948Sensor::calibrateIMU() {
    Serial.println("Calibrating IMU, please keep it still...");
    int sample_count = 500;
    _acc_bias_x = _acc_bias_y = _acc_bias_z = 0.0;
    _gyro_bias_x = _gyro_bias_y = _gyro_bias_z = 0.0;
    
    unsigned long start_time = millis();
    int count = 0;
    while (count < sample_count && (millis() - start_time) < 5000) { // Timeout in 5s
        if (_icm.readSensor() == 1) {
            _acc_bias_x += _icm.getAccelX_mss();
            _acc_bias_y += _icm.getAccelY_mss();
            _acc_bias_z += _icm.getAccelZ_mss() - 9.81; // Assume gravity on Z
            _gyro_bias_x += _icm.getGyroX_rads();
            _gyro_bias_y += _icm.getGyroY_rads();
            _gyro_bias_z += _icm.getGyroZ_rads();
            count++;
        }
    }
    
    if (count > 0) {
        _acc_bias_x /= count;
        _acc_bias_y /= count;
        _acc_bias_z /= count;
        _gyro_bias_x /= count;
        _gyro_bias_y /= count;
        _gyro_bias_z /= count;
    }
    Serial.println("Calibration complete.");
}

void ICM20948Sensor::getEulerAngles(float &roll, float &pitch, float &yaw) {
    roll = _filter.getRoll();
    pitch = _filter.getPitch();
    yaw = _filter.getYaw();
}

float ICM20948Sensor::getRoll() {
    return _filter.getRoll();
}

float ICM20948Sensor::getPitch() {
    return _filter.getPitch();
}

float ICM20948Sensor::getYaw() {
    return _filter.getYaw();
}

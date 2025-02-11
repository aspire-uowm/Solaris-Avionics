/*
File: ICM20948SENSOR.cpp
Author: Theodoros Taloumtzis
Date: 10/02/2025
Purpose: Implements the ICM20948Sensor class functions.
*/

#include "ICM20948SENSOR.h"

// Constructor accepts an I2C instance
ICM20948Sensor::ICM20948Sensor(TwoWire* wire) : _wire(wire) {
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
ICM20948Sensor::~ICM20948Sensor() { }


void ICM20948Sensor::setup() {
    // Initialize serial communication
    Serial.begin(9600);

    _wire->begin();
    
    // Initialize the ICM-20948
    if (_icm.begin(*_wire, 0x69) != ICM_20948_Stat_Ok) {
        Serial.println("ICM-20948 connection failed");
        while (1);
    }

    Serial.println("ICM-20948 connected successfully!");

    calculateIMUErrors();
}

void ICM20948Sensor::loop() {
    _icm.getAGMT(); // Read Accelerometer, Gyroscope, Magnetometer, and Temperature
    
    _ax = _icm.accX();
    _ay = _icm.accY();
    _az = _icm.accZ();
    
    _gx = _icm.gyrX();
    _gy = _icm.gyrY();
    _gz = _icm.gyrZ();

    // Convert raw accelerometer readings to g-force values (ICM-20948 uses different scale)
    _acc_x = _ax * 9.81 / 32768.0;
    _acc_y = _ay * 9.81 / 32768.0;
    _acc_z = _az * 9.81 / 32768.0;

    // Calculate roll and pitch angles
    _angle_x = atan2(_acc_y, _acc_z) * 180 / PI;
    _angle_y = atan2(-_acc_x, sqrt(_acc_y * _acc_y + _acc_z * _acc_z)) * 180 / PI;

    // Calculate yaw using gyroscope
    unsigned long current_time = millis();
    float elapsed_time = (current_time - _previous_time) / 1000.0;
    _previous_time = current_time;

    float gyro_z = _gz / 16.4; // ICM-20948 gyroscope sensitivity
    _angle_z += gyro_z * elapsed_time;

    // Print values
    Serial.print("AccX: "); Serial.println(_acc_x);
    Serial.print("AccY: "); Serial.println(_acc_y);
    Serial.print("AccZ: "); Serial.println(_acc_z);
    Serial.print("AngleX: "); Serial.println(_angle_x);
    Serial.print("AngleY: "); Serial.println(_angle_y);
    Serial.print("AngleZ: "); Serial.println(_angle_z);
}

// Getters
float ICM20948Sensor::getAccX() { return _acc_x; }
float ICM20948Sensor::getAccY() { return _acc_y; }
float ICM20948Sensor::getAccZ() { return _acc_z; }
float ICM20948Sensor::getAngleX() { return _angle_x; }
float ICM20948Sensor::getAngleY() { return _angle_y; }
float ICM20948Sensor::getAngleZ() { return _angle_z; }

void ICM20948Sensor::calculateIMUErrors() {
    int sample_count = 256;
    
    for (int i = 0; i < sample_count; i++) {
        _icm.getAGMT();
        
        _ax = _icm.accX();
        _ay = _icm.accY();
        _az = _icm.accZ();
        
        _gx = _icm.gyrX();
        _gy = _icm.gyrY();
        _gz = _icm.gyrZ();

        _acc_error_x += atan2(_ay, sqrt(_ax * _ax + _az * _az)) * 180 / PI;
        _acc_error_y += atan2(-_ax, sqrt(_ay * _ay + _az * _az)) * 180 / PI;

        _gyro_error_x += _gx / 16.4;
        _gyro_error_y += _gy / 16.4;
        _gyro_error_z += _gz / 16.4;
    }

    _acc_error_x /= sample_count;
    _acc_error_y /= sample_count;
    _gyro_error_x /= sample_count;
    _gyro_error_y /= sample_count;
    _gyro_error_z /= sample_count;

    Serial.print("AccErrorX: "); Serial.println(_acc_error_x);
    Serial.print("AccErrorY: "); Serial.println(_acc_error_y);
    Serial.print("GyroErrorX: "); Serial.println(_gyro_error_x);
    Serial.print("GyroErrorY: "); Serial.println(_gyro_error_y);
    Serial.print("GyroErrorZ: "); Serial.println(_gyro_error_z);
}

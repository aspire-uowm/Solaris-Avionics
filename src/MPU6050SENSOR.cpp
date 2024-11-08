#include "MPU6050SENSOR.h"
#include <math.h>


MPU6050Sensor::MPU6050Sensor(){

}

void MPU6050Sensor::setup(){
    // Initialize I2C connection
    Wire.begin();

    // Initialize the MPU6050 sensor
    _mpu.initialize();
    _mpu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

    calculateIMUErrors();

    // Check if the sensor connection is successful
    if (!_mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1); // Enter an infinite loop to prevent code execution
    }
}

void MPU6050Sensor::loop(){
    // Read the raw accelerometer and gyroscope data
    _mpu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

    // Convert raw accelerometer readings to g-force values
    _AccX = _ax / 16384.0;
    _AccY = _ay / 16384.0;
    _AccZ = _az / 16384.0;

    // Calculate the angles using accelerometer data
    _AngleX = atan2(_AccY, _AccZ) * 180 / PI;  // Roll angle in degrees
    _AngleY = atan2(-_AccX, sqrt(_AccY * _AccY + _AccZ * _AccZ)) * 180 / PI;  // Pitch angle in degrees

    // Calculate the approximate yaw angle using gyro data and time delta
    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - _previousTime) / 1000.0; // Convert to seconds
    _previousTime = currentTime;

    float GyroZ = _gz / 131.0; // Convert gyro Z to degrees/second
    _AngleZ += GyroZ * elapsedTime; // Integrate for yaw angle (AngleZ)


    // Print the error values to the Serial Monitor for reference
    Serial.print("AccX: ");
    Serial.println(_AccX);
    Serial.print("AccY: ");
    Serial.println(_AccY);
    Serial.print("AccZ: ");
    Serial.println(_AccZ);
    Serial.print("AngleX: ");
    Serial.println(_AngleX);
    Serial.print("AngleY: ");
    Serial.println(_AngleY);
    Serial.print("AngleZ: ");
    Serial.println(_AngleZ);
}

// Getters
float MPU6050Sensor::getAccX(){
    /* rerutns the g-force value for the X-Axis*/
    return _AccX;
}
float MPU6050Sensor::getAccY(){
    /* rerutns the g-force value for the Y-Axis*/
    return _AccY;
}
float MPU6050Sensor::getAccZ(){
    /* rerutns the g-force value for the Z-Axis*/
    return _AccZ;
}

float MPU6050Sensor::getAngleX(){
    /* rerutns the angle value for the X-Axis*/
    return _AngleX;
}
float MPU6050Sensor::getAngleY(){
    /* rerutns the angle value for the Y-Axis*/
    return _AngleY;
}
float MPU6050Sensor::getAngleZ(){
    /* rerutns the angle value for the Z-Axis*/
    return _AngleZ;
}



void MPU6050Sensor::calculateIMUErrors() {
    int sampleCount = 256; // Number of samples to average
    for (int i = 0; i < sampleCount; i++) {
        // Read raw accelerometer and gyroscope data
        _mpu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

        // Calculate accelerometer angles and sum them for averaging
        _AccX = _ax / 16384.0;
        _AccY = _ay / 16384.0;
        _AccZ = _az / 16384.0;

        _AccErrorX += atan2(_AccY, sqrt(_AccX * _AccX + _AccZ * _AccZ)) * 180 / PI;
        _AccErrorY += atan2(-_AccX, sqrt(_AccY * _AccY + _AccZ * _AccZ)) * 180 / PI;

        // Sum gyroscope readings for averaging
        _GyroErrorX += _gx / 131.0;
        _GyroErrorY += _gy / 131.0;
        _GyroErrorZ += _gz / 131.0;
    }

    // Average the error values
    _AccErrorX /= sampleCount;
    _AccErrorY /= sampleCount;
    _GyroErrorX /= sampleCount;
    _GyroErrorY /= sampleCount;
    _GyroErrorZ /= sampleCount;

    // Print the error values to the Serial Monitor for reference
    Serial.print("AccErrorX: ");
    Serial.println(_AccErrorX);
    Serial.print("AccErrorY: ");
    Serial.println(_AccErrorY);
    Serial.print("GyroErrorX: ");
    Serial.println(_GyroErrorX);
    Serial.print("GyroErrorY: ");
    Serial.println(_GyroErrorY);
    Serial.print("GyroErrorZ: ");
    Serial.println(_GyroErrorZ);
}
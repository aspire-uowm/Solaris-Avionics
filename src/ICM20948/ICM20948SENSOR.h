/*  
File: ICM20948SENSOR.h
Author: Theodoros Taloumtzis
Date: 10/02/2025
Purpose: Extends the ICM20948Sensor class with quaternion calculations, absolute yaw, and calibration.
*/

#ifndef ICM20948SENSOR_H
#define ICM20948SENSOR_H

#include <Wire.h>
#include "ICM20948.h"
#include <MadgwickAHRS.h>


class ICM20948Sensor {
  private:
    ICM20948 _icm;  // ICM-20948 sensor instance
    TwoWire* _wire;  // Reference to the I2C bus instance

    float _ax, _ay, _az, _gx, _gy, _gz, _mx, _my, _mz;
    float _acc_x, _acc_y, _acc_z;
    float _angle_x, _angle_y, _angle_z;
    float _velocity_x, _velocity_y, _velocity_z;
    unsigned long _previous_time;

    // Error correction values
    float _acc_bias_x, _acc_bias_y, _acc_bias_z;
    float _gyro_bias_x, _gyro_bias_y, _gyro_bias_z;

    // Quaternion values
    float _q0, _q1, _q2, _q3;

  public:
    ICM20948Sensor(TwoWire* wire);  // Constructor
    ~ICM20948Sensor();
    
    void setup();  // Initializes the sensor
    void loop();   // Runs in the loop function and updates sensor data
    void calibrateIMU();  // Method to calculate sensor errors

    // Velocity tracking
    void updateVelocity();
    void resetVelocity();

    // Getters for angles
    float getAngleX();
    float getAngleY();
    float getAngleZ();

    // Getters for velocity
    float getVelocityX();
    float getVelocityY();
    float getVelocityZ();

    // Getters for quaternion values
    float getQuaternionQ0();
    float getQuaternionQ1();
    float getQuaternionQ2();
    float getQuaternionQ3();

    // Compute Euler angles from quaternion
    void getEulerAngles(float &roll, float &pitch, float &yaw);
};

#endif
/*  
File: ICM20948SENSOR.h
Author: Theodoros Taloumtzis
Date: 10/02/2025
Purpose: Implements the ICM20948Sensor class and its API class declarations.
*/

#ifndef ICM20948SENSOR_H
#define ICM20948SENSOR_H

#include <Wire.h>
#include "ICM_20948.h"

class ICM20948Sensor {
  private:
    ICM_20948_I2C _icm;  // ICM-20948 sensor instance
    TwoWire* _wire;  // Reference to the I2C bus instance

    float _ax, _ay, _az, _gx, _gy, _gz;

    // Variables to store the sensor values 
    float _acc_x, _acc_y, _acc_z;
    float _angle_x, _angle_y, _angle_z;
    unsigned long _previous_time;

    // Variables to store sensor error values
    float _acc_error_x, _acc_error_y;
    float _gyro_error_x, _gyro_error_y, _gyro_error_z;

  public:
    ICM20948Sensor(TwoWire* wire);  // Constructor accepts I2C instance reference
    ~ICM20948Sensor();
    
    void setup();  // Initializes the sensor

    void loop();   // Runs in the loop function and updates sensor data

    void calculateIMUErrors();  // Method to calculate sensor errors

    // Getters
    float getAccX();
    float getAccY();
    float getAccZ();

    float getAngleX();
    float getAngleY();
    float getAngleZ();
};

#endif

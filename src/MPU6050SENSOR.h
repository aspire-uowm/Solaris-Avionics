#ifndef MPU6050SENSOR_H
#define MPU6050SENSOR_H

#include <Wire.h>
#include <MPU6050.h>


class MPU6050Sensor {
  private:
    MPU6050 _mpu;
    int16_t _ax, _ay, _az, _gx, _gy, _gz;

    // Variables to store the sensor values 
    float _AccX, _AccY, _AccZ;
    float _AngleX, _AngleY, _AngleZ;
    unsigned long _previousTime = 0;


    // Variables to store sensor error values
    float _AccErrorX = 0, _AccErrorY = 0;
    float _GyroErrorX = 0, _GyroErrorY = 0, _GyroErrorZ = 0;

  public:
    MPU6050Sensor();
    void setup();

    // Runs in the loop function and updates the sensor data.
    void loop();

    // Method to calculate sensor errors
    void calculateIMUErrors(); 

    // Getters
    float getAccX();
    float getAccY();
    float getAccZ();

    float getAngleX();
    float getAngleY();
    float getAngleZ();
};

#endif 
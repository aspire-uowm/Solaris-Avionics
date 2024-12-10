#ifndef MPU6050SENSOR_H
#define MPU6050SENSOR_H

#include <Wire.h>
#include <MPU6050.h>


class MPU6050Sensor {
  private:
    MPU6050 _mpu;
    int16_t _ax, _ay, _az, _gx, _gy, _gz;

    // Variables to store the sensor values 
    float _acc_x, _acc_y, _acc_z;
    float _angle_x, _angle_y, _angle_z;
    unsigned long _previous_time;


    // Variables to store sensor error values
    float _acc_error_x, _acc_error_y;
    float _gyro_error_x, _gyro_error_y, _gyro_error_z;

  public:
    MPU6050Sensor();
	~MPU6050Sensor();
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
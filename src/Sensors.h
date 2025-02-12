/*  
File: Sensors.h
Author: Theodoros Taloumtzis
Date: 12/02/2025
Purpose: Implements the an agregation class that uses the reduntand sensors to save the the averaged sensor data.
*/

#ifndef SENSOR_H
#define SENSOR_H

#include "ICM20948\ICM20948SENSOR.h"
#include "BMP\BMPSensor.h"


class Sensors
{
private:

    ICM20948Sensor* _icm1;
    BMPSensor* _bmp1;
    BMPSensor* _bmp2;

    float _avg_altitude;
    float _avg_temperature;
    float _avg_pressure;

    float _avg_acc_x;
    float _avg_acc_y;
    float _avg_acc_z;

    float _avg_angle_x;
    float _avg_angle_y;
    float _avg_angle_z;

public:

// Constructor
Sensors(ICM20948Sensor* icm, BMPSensor* bmp1, BMPSensor* bmp2);

// Destructor
~Sensors();

// Methods
void setup();

void loop();

void update_avg_sensor_data();

// Getters
float get_avg_altitude();
float get_avg_temperature();
float get_avg_pressure();

float get_avg_acc_x();
float get_avg_acc_y();
float get_avg_acc_z();

float get_avg_angle_x();    
float get_avg_angle_y();
float get_avg_angle_z();

};



#endif // SENSOR_H
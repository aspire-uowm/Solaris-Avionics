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
#include "DHT22\DHT22Sensor.h"


class Sensors
{
private:

    ICM20948Sensor* _icm1;
    BMPSensor* _bmp1;
    BMPSensor* _bmp2;
    DHTSensor* _dht;

    float _avg_altitude;
    float _avg_temperature;
    float _avg_pressure;
    float _avg_humidity;

    float _avg_roll;
    float _avg_pitch;
    float _avg_yaw;


public:

// Constructor
Sensors(ICM20948Sensor* icm, BMPSensor* bmp1, BMPSensor* bmp2, DHTSensor* dht);

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
float get_avg_humidity();

float get_u_pitot(){ // pitot tube velocity
    return 0; // to be implemented
}

float get_avg_roll();

float get_avg_pitch();

float get_avg_yaw();

// float get_avg_acc_x();
// float get_avg_acc_y();
// float get_avg_acc_z();

// float get_avg_angle_x();    
// float get_avg_angle_y();
// float get_avg_angle_z();

// float get_avg_velocity_x();    
// float get_avg_velocity_y();
// float get_avg_velocity_z();
 

};



#endif // SENSOR_H
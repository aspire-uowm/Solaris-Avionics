/*  
File: BMPSensor.h
Author: Theodoros Taloumtzis
Date: 10/12/2024
Purpose: Implements the BMPSensor class declarations.
*/

#ifndef BMP_SENSOR_H
#define BMP_SENSOR_H

#include <Wire.h>
#include <Adafruit_BMP3XX.h>

class BMPSensor{
private:
    float _temperature;
    float _pressure;
    float _altitude;

    Adafruit_BMP3XX _bmp;
    bool _initialized;

    float _temp_error;
    float _pressure_error;
    float _altitude_error;

    const float SEALEVELPRESSURE_HPA = 1013.25;

public:
    // Methods
    BMPSensor();
    ~BMPSensor();
    void setup();
    void loop();
    void calculateBMPErrors();

    // Getters
    float geTemperature();
    float getPressure();
    float getAltitute();
};

#endif // BMP_SENSOR_H
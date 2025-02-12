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
    TwoWire* _wire;
    byte _address; 
    float _temperature;
    float _pressure;
    float _altitude;

    Adafruit_BMP3XX _bmp;
    bool _initialized;

    float _temp_error;
    float _pressure_error;
    float _altitude_error;

    const float SEALEVELPRESSURE_HPA =  1027.20; // in hecto pascal in kozani greece

public:
    // Methods
    BMPSensor(byte addresss, TwoWire* wire);
    ~BMPSensor();
    void setup();
    void loop();
    void calculateBMPErrors();

    // Getters
    float getTemperature();
    float getPressure();
    float getAltitute();
};

#endif // BMP_SENSOR_H
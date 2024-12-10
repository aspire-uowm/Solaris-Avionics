/*  
File: BMPSensor.cpp
Author: Theodoros Taloumtzis
Date: 10/12/2024
Purpose: Implements the BMPSensor class functions.
*/ 

#include "BMPSensor.h"
#include <cmath>

#define SEALEVELPRESSURE_HPA (1013.25)

BMPSensor::BMPSensor(){
    _temp_error = 0.0;
    _pressure_error = 0.0;
    _altitude_error = 0.0;
    _temperature = 0.0;
    _pressure = 0.0;
    _altitude = 0.0;
    _initialized = false;
}

void BMPSensor::setup(){
    // Initialize serial communication
    Serial.begin(9600);

    // Initialize I2C communication
    Wire.begin();

    if (!_bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
        Serial.println("Could not find a valid BMP388 sensor, check wiring!");
        while (1);
    }

    // Set up oversampling and filter initialization
    _bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    _bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    _bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    _bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    calculateBMPErrors();
}

void BMPSensor::loop(){
    if (! _bmp.performReading()) { // if reading fails
        Serial.println("Failed to perform reading :(");
        return;
    }

    _temperature = _bmp.temperature; 
    Serial.print("[BMP388]Temperature = ");
    Serial.print(_temperature);
    Serial.println(" *C");

    _pressure = _bmp.pressure;
    Serial.print("[BMP388]Pressure = ");
    Serial.print(_pressure);
    Serial.println(" Pa");

    _altitude = _bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("[BMP388]Approx. Altitude = ");
    Serial.print(_altitude);
    Serial.println(" m");
}



// Getters
float BMPSensor::geTemperature(){
    return _temperature;
}

float BMPSensor::getPressure(){
    return _pressure;
}

float BMPSensor::getAltitute(){
    return _altitude;
}

void BMPSensor::calculateBMPErrors(){
    int samplecount = 200;
    for(int i = 0; i<samplecount; i++){
        if(! _bmp.performReading()) { // if reading fails
            Serial.println("Failed to perform reading :(");
            return;
        }

        _temp_error += _bmp.temperature;
        _pressure_error += _bmp.pressure;
        _altitude_error += _bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }

    if (! _bmp.performReading()) { // if reading fails
        Serial.println("Failed to perform reading :(");
        return;
    }


    // calculatin the RMSE of the sensors
    float temp_ref = _bmp.temperature;
    float pressure_ref = _bmp.pressure;
    float altitude_ref = _bmp.readAltitude(SEALEVELPRESSURE_HPA);

    temp_ref = temp_ref * samplecount;
    pressure_ref = pressure_ref * samplecount;
    altitude_ref = altitude_ref * samplecount;
    
    _temp_error =  sqrt(pow(_temp_error - temp_ref, 2)/samplecount);
    // Serial.print("Temperature Error = ");
    // Serial.print(_temp_error);
    // Serial.println(" *C");
    _pressure_error = sqrt(pow(_pressure_error - pressure_ref, 2)/samplecount);
    // Serial.print("Pressure Error = ");
    // Serial.print(_pressure_error);
    // Serial.println(" Pa");
    _altitude_error = sqrt(pow(_altitude_error - altitude_ref, 2)/samplecount);
    // Serial.print("Altitude Error = ");
    // Serial.print(_altitude_error);
    // Serial.println(" m");
}

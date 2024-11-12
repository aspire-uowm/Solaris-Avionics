#include "MPXV7002DP.h"
#include <iostream>

// Constructor: Initializes the analog pin and sets the reference voltage
PressureSensor::PressureSensor(int analogPin, float density) : 
                _analogPin(analogPin), _referenceVoltage(3.3), _dividerRation(0.735), _density(density) {} //

// Initializes the sensor (if any additional setup is needed)
void PressureSensor::begin() {
    pinMode(_analogPin, INPUT);
}

// Reads the differential pressure in kPa
float PressureSensor::readPressure() {
    // Read the raw analog value
    int analogValue = analogRead(_analogPin);
    
    // Convert raw value to voltage
    float scaledVoltage = analogValue * (_referenceVoltage / 1023.0);

    float sensorVoltage = scaledVoltage / _dividerRation;
    
    // Calculate pressure in kPa based on sensor transfer function
    float pressure = (sensorVoltage - 2.5) ;  // From datasheet formula
    
    return pressure;
}

float PressureSensor::airspeed(){
    float pressure = readPressure();
    float airspeed = sqrt(2 * pressure/_density);
    return airspeed;
}
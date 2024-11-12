#include "MPXV7002DP.h"
#include <iostream>

// Constructor: analog pin init and ref voltage based on the esp32 handle
PressureSensor::PressureSensor(int analogPin, float density) : 
                _analogPin(analogPin), _referenceVoltage(3.3), _dividerRation(0.735), _density(density) {} //

// Sensor init
void PressureSensor::begin() {
    pinMode(_analogPin, INPUT);
}

// diff pressure read
float PressureSensor::readPressure() {
    // Read the analog value
    int analogValue = analogRead(_analogPin);
    
    // Convert analogRead to voltage based on the 3.3V ref
    float scaledVoltage = analogValue * (_referenceVoltage / 1023.0);

    float sensorVoltage = scaledVoltage / _dividerRation;
    
    // Calculate diff pressure
    float pressure = (sensorVoltage - 2.5) ;  // datasheet formula
    
    return pressure;
}

float PressureSensor::airspeed(){
    float pressure = readPressure();
    float airspeed = sqrt(2 * pressure/_density);
    return airspeed;
}
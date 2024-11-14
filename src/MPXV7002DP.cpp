#include "MPXV7002DP.h"
//#include <iostream>

// Constructor: analog pin init and ref voltage based on the esp32 handle
PressureSensor::PressureSensor(int analogPin, float density) : 
                _analogPin(analogPin), _referenceVoltage(3.3), _dividerRation(0.735), _density(density) {


                } //

// Sensor init
void PressureSensor::begin() {
    pinMode(_analogPin, INPUT);
}

// diff pressure read
float PressureSensor::readPressure() {
    // Read the analog value
    int analogValue = analogRead(_analogPin);
    
    // Convert analogRead to voltage based on the 3.3V ref
    float scaledVoltage = analogValue * (_referenceVoltage / 4096.0);

    float sensorVoltage = scaledVoltage / _dividerRation;
    
    // Calculate diff pressure
    float diff_pressure = (sensorVoltage - 2.5) ;  // datasheet formula
    if(diff_pressure < 0){
        diff_pressure = -diff_pressure; //flag is needed for minus sign
    }

    return diff_pressure;
}



float PressureSensor::airspeed(){
    float diff_pressure = readPressure();
    float airspeed = sqrt(2 * diff_pressure/_density); //density read
    return airspeed;
}
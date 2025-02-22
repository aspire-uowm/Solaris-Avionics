#include "PRESSURE_SENSOR.h"


// Constructor
PressureSensor::PressureSensor(int analogPin, int analogRefPin , float density) :
                _analogPin(analogPin), _analogRefPin(analogRefPin), _density(density) {}

// Destructor
PressureSensor::~PressureSensor() {
    // empty
}


// Sensor init
void PressureSensor::setup() {
    analogReadResolution(12); // 12 bit resolution
    pinMode(_analogPin, INPUT);
    pinMode(_analogRefPin, INPUT);

    // Set the variable values

    _airSpeed = 0.0;
    _Vout = 0.0;
    _Vref = 0.0;
    _diffPressure = 0.0;

}


void PressureSensor::loop(){
    _Vref = analogRead(_analogRefPin) * 3.3 / ADC_RESOLUTION; // 3.3V is the reference voltage
    Serial.print("Vref: "); Serial.println(_Vref);
    updatePressure();
    speedUpdate();
}

// diff pressure read
void PressureSensor::updatePressure() {
    _Vout = analogRead(_analogPin) * _Vref / ADC_RESOLUTION; // 3.3V is the reference voltage
    Serial.print("Vout: "); Serial.println(_Vout);
    _diffPressure = (_Vout - 0.1 * _Vref) *(P_MAX - P_MIN) / (0.8 * _Vref) + P_MIN;
    Serial.print("Diff Pressure: "); Serial.println(_diffPressure);
    _diffPressure = _diffPressure / 10; // convert to kPa form mbar
}



void PressureSensor::speedUpdate(){
    _diffPressure = abs(_diffPressure);
    int flag = (_diffPressure < 0) ? -1 : 1;

    _airSpeed = sqrt(2 * _diffPressure/_density); //density calculation

    _airSpeed = _airSpeed * flag;

}

float PressureSensor::getSpeed(){
    return _airSpeed;
}

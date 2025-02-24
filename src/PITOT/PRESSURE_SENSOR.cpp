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
    _bias = 0.0;
    _kalmanX = 0.0;
    _kalmanP = 1.0;
    _kalmanQ = 0.01;
    _kalmanR = 0.1;
    Serial.println("Calibrating Pitot Bias.");
    calibrateBias();
    

}


void PressureSensor::loop(){
    _Vref = analogRead(_analogRefPin) * 3.3 / ADC_RESOLUTION; // 3.3V is the reference voltage
    Serial.print("Vref: "); Serial.println(_Vref);
    updatePressure();
    speedUpdate();
}

float PressureSensor::applyKalmanFilter(float measurement){
    // Prediction step
    _kalmanP += _kalmanQ;

    // Measurement update step
    float K = _kalmanP / (_kalmanP + _kalmanR); // Kalman gain
    _kalmanX = _kalmanX + K * (measurement - _kalmanX);
    _kalmanP = (1 - K) * _kalmanP;

    return _kalmanX;
}

// diff pressure read
void PressureSensor::updatePressure() {
    _Vout = analogRead(_analogPin) * _Vref / ADC_RESOLUTION; // 3.3V is the reference voltage
    Serial.print("Vout: "); Serial.println(_Vout);
    _diffPressure = (_Vout - 0.1 * _Vref) *(P_MAX - P_MIN) / (0.8 * _Vref) + P_MIN;
    Serial.print("Diff Pressure: "); Serial.println(_diffPressure);
    _diffPressure -= _bias;
    _diffPressure = _diffPressure * 100; // convert mbar to Pa


    // Apply Kalman filtering
    //_diffPressure = applyKalmanFilter(_diffPressure);
    
}

void PressureSensor::calibrateBias()
{
    int interval = 10;
    int calibrationTime = 4000;
    int samples = 400;

    float Vref = analogRead(_analogRefPin) * 3.3 / ADC_RESOLUTION;
    float Vout = analogRead(_analogPin) * Vref / ADC_RESOLUTION;
    
    float ref = (Vout - 0.1 * Vref) * (P_MAX - P_MIN) / (0.8 * Vref) + P_MIN;
    float sum = 0;
    unsigned long startTime = millis();
    unsigned long lastSampleTime = 0;
    int validSamples = 0;

    while (millis() - startTime < calibrationTime) {
        if (millis() - lastSampleTime >= interval) {  // Sample every 20ms
            lastSampleTime = millis();

            Vref = analogRead(_analogRefPin) * 3.3 / ADC_RESOLUTION;
            Vout = analogRead(_analogPin) * Vref / ADC_RESOLUTION;

            sum += (Vout - 0.1 * Vref) * (P_MAX - P_MIN) / (0.8 * Vref) + P_MIN - ref;
            validSamples +=1;
            if (validSamples >= samples) break;  // Stop if we reach 200 valid samples
        }
    }
    
    _bias = sum /samples;

    Serial.print("Bias: "); Serial.println(_bias);
    
}

void PressureSensor::speedUpdate()
{
    int flag = (_diffPressure < 0) ? -1 : 1;
    _diffPressure = abs(_diffPressure);

    _airSpeed = sqrt(2 * _diffPressure/_density); //density calculation

    _airSpeed = _airSpeed * flag;

}

float PressureSensor::getSpeed(){
    return _airSpeed;
}

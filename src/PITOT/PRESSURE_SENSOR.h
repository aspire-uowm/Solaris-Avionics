#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Arduino.h>

#define P_MIN -600
#define P_MAX 600
#define ADC_RESOLUTION 4095

#define NUM_SMAPLES 10 
#define ALPHA 0.1




class PressureSensor {
public:
    // Constructor
    PressureSensor(int analogPin,int analogRefPin, float density);

    ~PressureSensor();

    // sensor init
    void setup();

    void loop();

    float applyKalmanFilter(float mesurement);

    void calibrateBias();
    void updatePressure();
    void speedUpdate();

    float getSpeed();

private:
    int _analogPin;  
    int _analogRefPin;
    float _diffPressure;   
    float _Vout;
    float _Vref;
    float _density;
    float _airSpeed;
    float _bias;


    // Kalman filter variables
    float _kalmanX; // Estimated state (pressure)
    float _kalmanP; // Error covariance
    float _kalmanQ; // Process noise covariance
    float _kalmanR; // Measurement noise covariance
};

#endif // PRESSURE_SENSOR_H
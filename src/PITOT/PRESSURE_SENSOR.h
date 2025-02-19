#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Arduino.h>

#define P_MIN -600
#define P_MAX 600
#define ADC_RESOLUTION 4095


class PressureSensor {
public:
    // Constructor
    PressureSensor(int analogPin,int analogRefPin, float density);

    ~PressureSensor();

    // sensor init
    void setup();

    void loop();

    
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
};

#endif // PRESSURE_SENSOR_H
#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Arduino.h>

class PressureSensor {
public:
    // Constructor
    PressureSensor(int analogPin, float density);

    // sensor init
    void begin();

    // current kPa read
    float readPressure();

    float airspeed();

private:
    int _analogPin;       // esp32 pin that is connected to the vout of the sensor
    float _referenceVoltage; // 3.3V or 5V
    float _dividerRation;
    float _density;
};

#endif // PRESSURE_SENSOR_H
#ifndef SPEEDPRESSURE_H
#define SPEEDPRESSURE_H

#include <Arduino.h>


#define PRESSURE_SEA_LEVEL 101325.0    
#define ABSOLUTE_0_KELVIN -273.15    
#define DRY_AIR_MOLAR_MASS 0.0289644
#define UNIVERSAL_GAS_CONSTANT 8.314

class Pressure {
public:
    // Constructor
    Pressure();

    void begin(int baudrate);

    int Init(int pin);

    float GetAirSpeed();

    void loop();

private:
    int _pin;               
    int ref_pressure;       
    float pitotpressure;    
    float ambientpressure;  
    float temperature;      
    float density;         
    float airspeed_ms;      
    float airspeed_kmh;    
};

#endif // SPEEDPRESSURE_H


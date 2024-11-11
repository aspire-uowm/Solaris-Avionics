#include "MPXV7002DP.h"

Pressure::Pressure() {
    
}

void Pressure::begin(int baudRate) {
    Serial.begin(baudRate);
}

int Pressure::Init(int pin) {
    _pin = pin;
    pinMode(_pin, INPUT);
    ref_pressure = analogRead(_pin);
    for (int i = 1; i <= 200; i++) {
        ref_pressure = (analogRead(_pin)) * 0.25 + ref_pressure * 0.75;
        delay(20); //to calculate an accurate value of ref pressure (static pressure inside the rocket)
    }
    return ref_pressure;
}

float Pressure::GetAirSpeed() {
    int air_pressure = 0;
    for (int i = 0; i < 8; i++)
        air_pressure += analogRead(_pin);
    air_pressure >>= 3; //shift for mean value /2^3

    if (air_pressure < ref_pressure)
        air_pressure = ref_pressure;

    pitotpressure = 5000.0f * ((air_pressure - ref_pressure) / 1024.0f) + PRESSURE_SEA_LEVEL;
    ambientpressure = PRESSURE_SEA_LEVEL;
    temperature = 20.0f + ABSOLUTE_0_KELVIN;
    density = (ambientpressure * DRY_AIR_MOLAR_MASS) / (temperature * UNIVERSAL_GAS_CONSTANT);
    airspeed_ms = sqrt((2 * (pitotpressure - ambientpressure)) / density);
    airspeed_kmh = airspeed_ms * 3.600;

    return airspeed_kmh;
}


void Pressure::loop(){
    
  Serial.println("Airspeed is:");
  Serial.println(GetAirSpeed());
  Serial.println("km/h");
}



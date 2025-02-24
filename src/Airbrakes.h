#ifndef AIRBRAKES_H
#define AIRBRAKES_H

#include <Arduino.h>
#include "Servo\PWMServo.h" 
#include "Sensors.h"
#include <math.h>  // For math operations


class Airbrakes {
  private:
    int pin;  // Pin that controls the airbrakes (e.g., PWM or digital output)
    PWMServo _myServo;
    Sensors *_sensors;
    //Servo myServo;
    float _Aref ;
    float _m ; 
    float _c1;
    float _c2;
    float _T;
    float _k;
    float _H;
    int _airbrake_extension;
  public:
    // Constructor to initialize the airbrakes with the pin number
    Airbrakes(int controlPin, Sensors* sensorsData);

    // Method to activate the airbrakes
    void deploy();

    // Method to deactivate the airbrakes
    void retract();

    // Method to adjust the position of the airbrakes (e.g., open/close)
    void adjust(int angle);

    void control_based_on_sensors();

    void update_airbrake_extension(int angle);

    void pid_control();
    
    float calculate_apogee();

    void loop();

    void setup();
};

#endif

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
    
    float calculate_apogee();

    void loop();

    void setup();
};

#endif











/*

#ifndef AIRBRAKES_H
#define AIRBRAKES_H

#include <Arduino.h>
#include <Servo.h>  // Include the Servo library to control the servo motor
#include "Sensors.h" // Include the Sensors class to access sensor data

class Airbrakes {
  private:
    int pin;       // Pin that controls the servo motor
    Servo myServo; // Create a Servo object
    Sensors* sensors;  // Pointer to the Sensors object

  public:
    // Constructor to initialize the airbrakes with the pin number and sensors object
    Airbrakes(int controlPin, Sensors* sensorData);

    // Method to deploy the airbrakes fully (180 degrees)
    void deploy();

    // Method to retract the airbrakes to 0 degrees
    void retract();

    // Method to adjust the airbrakes to a specific angle (0 to 180 degrees)
    void adjust(int angle);

    // Method to control the airbrakes based on sensor data
    void control_based_on_sensors();
};

#endif
*/

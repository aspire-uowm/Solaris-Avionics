#include "Airbrakes.h"


Airbrakes::Airbrakes(int controlPin) {
  pin = controlPin;  // Initialize the pin number
}

void Airbrakes::deploy() {
  // Logic to activate the airbrakes (e.g., set the pin high)
  myServo.write(180);  // Turn on airbrakes
  delay(1000);
}

void Airbrakes::retract() {
  // Logic to deactivate the airbrakes (e.g., set the pin low)
  myServo.write(0);  // Turn off airbrakes
  delay(1000);
}

void Airbrakes::adjust(int angle) {
  // Adjust the airbrakes to a specific angle between 0 and 180 degrees
  if (angle >= 0 && angle <= 180) {
    myServo.write(angle); // Set the servo to the desired angle
    delay(1000);           // Wait for the servo to reach the position (optional)
  }
}

// Loop method for continuously controlling the airbrakes
void Airbrakes::loop() {
    Serial.println("Airbrakes loop started.");
    deploy();       // Deploy airbrakes fully to 180 degrees
    delay(5000);    // Wait for 5 seconds
  
    retract();      // Retract the airbrakes back to 0 degrees
    delay(5000);    // Wait for 5 seconds
  
    adjust(90);     // Adjust airbrakes to 90 degrees
    delay(5000);    // Wait for 5 seconds
  
    adjust(45);     // Adjust airbrakes to 45 degrees
    delay(5000);    // Wait for 5 seconds
  }

void Airbrakes::setup(){
    // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myServo.setPeriodHertz(50);    // standard 50 hz servo
	myServo.attach(pin);
    Serial.println("Airbrakes setup complete.");
}












/*


#include "Airbrakes.h"

Airbrakes::Airbrakes(int controlPin, Sensors* sensorData) {
  pin = controlPin;          // Initialize the pin number
  myServo.attach(pin);       // Attach the servo to the specified pin
  sensors = sensorData;      // Assign the sensors object to get sensor data
}

void Airbrakes::deploy() {
  // Deploy the airbrakes to 180 degrees (fully opened)
  myServo.write(180);  // Move the servo to 180 degrees
  delay(1000);         // Wait for the servo to reach the position (optional)
}

void Airbrakes::retract() {
  // Retract the airbrakes to 0 degrees (fully closed)
  myServo.write(0);    // Move the servo back to 0 degrees
  delay(1000);         // Wait for the servo to reach the position (optional)
}

void Airbrakes::adjust(int angle) {
  // Adjust the airbrakes to a specific angle between 0 and 180 degrees
  if (angle >= 0 && angle <= 180) {
    myServo.write(angle); // Set the servo to the desired angle
    delay(1000);           // Wait for the servo to reach the position (optional)
  }
}

void Airbrakes::control_based_on_sensors() {
  // Get the sensor data
  float altitude = sensors->get_avg_altitude();
  float acc_x = sensors->get_avg_acc_x();
  float angle_x = sensors->get_avg_angle_x();

  // Conditions to deploy the airbrakes
  if (altitude > 100.0 || acc_x > 3.0 || abs(angle_x) > 30.0) {
    deploy();  // Deploy the airbrakes if altitude is above 100 meters, acceleration is high, or angle is steep
  }
  else {
    retract();  // Retract the airbrakes if the conditions are not met
  }
}
  */

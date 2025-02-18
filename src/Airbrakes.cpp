#include "Airbrakes.h"


#define MAX_BRAKE_ANGLE 180               // Maximum deployment angle
#define MIN_BRAKE_ANGLE 0                 // Minimum deployment angle
#define ALTITUDE_DEPLOY_THRESHOLD 100.0  // Adjust as needed
#define G 9.81                             // Gravitational acceleration (m/s^2)
#define cd 1.1649385
#define p  1.225 
#define m 70  //Temporary
#define Aref 2268 //Temporary


// PID variables
float kp = 1.0, ki = 0.1, kd = 0.05;  // Tune these parameters
float prev_error = 0.0;
float integral = 0.0;
unsigned long last_time = 0;

Airbrakes::Airbrakes(int controlPin,Sensors * sensorsData) {
  pin = controlPin;  // Initialize the pin number
  _sensors = sensorsData;  // Assign the sensors object
}

void Airbrakes::deploy() {
  // Logic to activate the airbrakes (e.g., set the pin high)
  Serial.println("Deploying airbrakes...");
  _myServo.write(MAX_BRAKE_ANGLE);  // Turn on airbrakes
  delay(1000);
}

void Airbrakes::retract() {
  // Logic to deactivate the airbrakes (e.g., set the pin low)
  Serial.println("Retracting airbrakes...");
  _myServo.write(MIN_BRAKE_ANGLE);  // Turn off airbrakes
  delay(1000);
}

void Airbrakes::adjust(int angle) {
  // Adjust the airbrakes to a specific angle between 0 and 180 degrees
  if (angle >= MIN_BRAKE_ANGLE && angle <= MAX_BRAKE_ANGLE) {
    Serial.print("Adjusting airbrakes to angle: ");
    Serial.println(angle);
    _myServo.write(angle); // Set the servo to the desired angle
    delay(1000);           // Wait for the servo to reach the position (optional)
  }
  else{
    Serial.println("Invalid angle. Airbrakes not adjusted.");
  }
}

float Airbrakes::calculate_apogee(){
  float h = _sensors->get_avg_altitude();  // Current altitude
  float u_pitot = _sensors->get_u_pitot();     // Free-stream velocity
  float theta = _sensors->get_avg_pitch(); // Pitch angle

  //Drag coefficient constant
  k = (p * cd * Aref)/(2 * std::sin(theta));
  // Constants
  c1 = sqrt((m * G) / k);     // c1 value
  c2 = sqrt(m / (k * G));     // c2 value

  // Time to reach apogee
  T = (c2 * (atan(u_pitot * sin(theta)) / c1));
  
  // Apogee height calculation
  H = (c1 * c2 * log(1 / cos(T / c2))) + h;

  Serial.print("Calculated Apogee: ");
  Serial.println(H);
  return H;
}

void Airbrakes::control_based_on_sensors(){
  float altitude = _sensors->get_avg_altitude();

  Serial.print("Altitude: "); Serial.println(altitude);

  if (altitude > ALTITUDE_DEPLOY_THRESHOLD) {
    deploy();
  }
  else{
    retract();
  }
}

// Loop method for continuously controlling the airbrakes
void Airbrakes::loop() {
    Serial.println("Airbrakes loop started.");
    control_based_on_sensors();
    delay(500);

    adjust(45);
  }

void Airbrakes::setup(){
    // Allow allocation of all timers
   // standard 50 hz servo
	_myServo.attach(pin);
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

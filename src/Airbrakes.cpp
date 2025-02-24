#include "Airbrakes.h"


#define MAX_BRAKE_ANGLE 180               // Maximum deployment angle
#define MIN_BRAKE_ANGLE 0                 // Minimum deployment angle
#define ALTITUDE_DEPLOY_THRESHOLD 100.0  // Adjust as needed
#define ALTITUDE_OF_GOAL 3000.0
#define G 9.81                             // Gravitational acceleration (m/s^2)
#define cd 1.1649385
#define p  1.225 
#define INITIAL_MASS 70.0  // Initial mass (kg)
#define FUEL_BURN_RATE 0.5 // Fuel burn rate (kg/s)

float rocket_mass = INITIAL_MASS;

// Airbrake Reference Areas (mm²)
#define _Aref0 21750      // 0 cm extend 
#define _Aref1 24549.8102 // 1 cm extend 
#define _Aref2 27349.6203 // 2 cm extend 
#define _Aref3 30009.44   // 2.95 cm (full deployment)

// PID variables
float kp = 1.0, ki = 0.1, kd = 0.05;  // Tune these parameters
float prev_error = 0.0;
float integral = 0.0;
unsigned long last_time = 0;

// Constructor
Airbrakes::Airbrakes(int controlPin,Sensors * sensorsData) {
  pin = controlPin;  // Initialize the pin number
  _sensors = sensorsData;  // Assign the sensors object
  _airbrake_extension = 0;   // Initialize airbrake extension
}

// Deploy Airbrakes (Full)
void Airbrakes::deploy() {
  // Logic to activate the airbrakes (e.g., set the pin high)
  Serial.println("Deploying airbrakes...");
  _myServo.write(MAX_BRAKE_ANGLE);  // Turn on airbrakes
  delay(1000);
}

// Retract Airbrakes
void Airbrakes::retract() {
  // Logic to deactivate the airbrakes (e.g., set the pin low)
  Serial.println("Retracting airbrakes...");
  _myServo.write(MIN_BRAKE_ANGLE);  // Turn off airbrakes
  delay(1000);
}

// Adjust Airbrake Angle (0-180 degrees)
void Airbrakes::adjust(int angle) {
  // Adjust the airbrakes to a specific angle between 0 and 180 degrees
  if (angle >= MIN_BRAKE_ANGLE && angle <= MAX_BRAKE_ANGLE) {
    Serial.print("Adjusting airbrakes to angle: ");
    Serial.println(angle);
    _myServo.write(angle); // Set the servo to the desired angle
    delay(500);           // Wait for the servo to reach the position (optional)
    update_airbrake_extension(angle); 
  }else{
    Serial.println("Invalid angle. Airbrakes not adjusted.");
  }
}

// Update _airbrake_extension Based on Servo Angle
void Airbrakes::update_airbrake_extension(int angle) {
  if (angle == 0) _airbrake_extension = 0;
  else if (angle <= 60) _airbrake_extension = 1;
  else if (angle <= 120) _airbrake_extension = 2;
  else _airbrake_extension = 3;
}

// Calculate Apogee Prediction
float Airbrakes::calculate_apogee(){
  float h = _sensors->get_avg_altitude();  // Current altitude
  float u_pitot = _sensors->get_velocity();     // Free-stream velocity
  float theta = _sensors->get_avg_pitch(); // Pitch angle

  // Select Airbrake Reference Area (_Aref)
  float _Aref;
  switch (_airbrake_extension) {
      case 0: _Aref = _Aref0; break;
      case 1: _Aref = _Aref1; break;
      case 2: _Aref = _Aref2; break;
      case 3: _Aref = _Aref3; break;
      default: _Aref = _Aref0; // Default to no extension if invalid
  }

  //Drag coefficient constant
  _k = (p * cd * _Aref)/(2 * std::sin(theta));
  // Constants
  _c1 = sqrt((_m * G) / _k);     // c1 value
  _c2 = sqrt(_m / (_k * G));     // c2 value

  // Time to reach apogee
  _T = (_c2 * (atan(u_pitot * sin(theta)) / _c1));
  
  // Apogee height calculation
  _H = (_c1 * _c2 * log(1 / cos(_T / _c2))) + h;

  Serial.print("Calculated Apogee: ");
  Serial.println(_H);
  return _H;
}

void Airbrakes::pid_control(){
  float current_altitude = _sensors->get_avg_altitude();
  _H = calculate_apogee();
  float error = _H - current_altitude;


//Για καλυτερο prediction and control MAYBE
  //_Η = calculate_apogee();
  //float blended_target = (0.7 * ALTITUDE_OF_GOAL) + (0.3 * _H);
  //float error = blended_target - current_altitude;



  unsigned long current_time = millis();
  float delta_time = (float)(current_time - last_time) / 1000.0; // Convert to seconds
  last_time = current_time;

  // Integral term update with windup prevention
  if (abs(error) < 500) {
    integral += error * delta_time;
  } else {
    integral = 0; 
  }
  integral = constrain(integral, -500, 500);


  float derivative = (error - prev_error) / delta_time;
  prev_error = error;

  //PID Output 
  float pid_output = (kp * error) + (ki * integral) + (kd * derivative);
  pid_output = constrain(pid_output, -200, 200);

  //Convert PID output to servo angle (0-180 degrees)
  int new_angle = constrain(map(pid_output, -200, 200, 0, 180), MIN_BRAKE_ANGLE, MAX_BRAKE_ANGLE); 

  Serial.print("PID Adjusting Airbrakes to: ");
  Serial.println(new_angle);

  adjust(new_angle);
}

// Airbrake Control Logic Based on Sensors
void Airbrakes::control_based_on_sensors(){
  float altitude = _sensors->get_avg_altitude();
  Serial.print("Altitude: "); 
  Serial.println(altitude);

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
  }

void Airbrakes::setup(){
	_myServo.attach(pin);
  Serial.println("Airbrakes setup complete.");
}



//For mass

/*void update_rocket_mass(unsigned long elapsed_time) {
    // Time in seconds
    float elapsed_seconds = elapsed_time / 1000.0;
    
    // Update the mass based on fuel burn rate
    rocket_mass = INITIAL_MASS - FUEL_BURN_RATE * elapsed_seconds;
    
    // Ensure mass does not go below 0
    rocket_mass = max(rocket_mass, 0.0);
    
    Serial.print("Current Rocket Mass: ");
    Serial.println(rocket_mass);
}*/
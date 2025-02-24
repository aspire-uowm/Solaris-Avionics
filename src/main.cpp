#include <Arduino.h>
#include <Wire.h>
//#include "GPS\GPSsens.h"
#include "PITOT\PRESSURE_SENSOR.h"
#include "ICM20948\ICM20948SENSOR.h"
#include "BMP\BMPSensor.h"
#include "Sensors.h"
#include "DHT22\DHT22Sensor.h"
#include "Airbrakes.h"
#include"Servo\PWMServo.h"

#define DHT22_PIN 5
#define SERVO_PIN 4 // TVCX in the PCB schematics
#define PITOT_PIN 25
#define REF_PIN 26

// PDN pins 
#define A0 14
#define A1 15
#define BACKUP_BATTERY_FLAG 21
#define SS_OUT 20

//RED PINS
#define RED 38
#define BLUE 39
#define GREEN 40

#define AIR_DENSITY 1.293
#define INTERVAL 1000
#define ADC_RESOLUTION 12
#define ADC_MAX 4095
#define VOLTAGE_THRESHOLD 3.2


ICM20948Sensor _icm1(&Wire);
BMPSensor _bmp1(0x77, &Wire);
BMPSensor _bmp2(0x76, &Wire);
DHTSensor _dht(DHT22_PIN);
PressureSensor _pitot(PITOT_PIN, REF_PIN, AIR_DENSITY); 

Sensors _sensors(&_icm1, &_bmp1, &_bmp2, &_dht, &_pitot);


uint32_t _interval = 0;
uint32_t _interval2 = 0;

Airbrakes _airbrakes(SERVO_PIN, &_sensors);  // Servo connected to pin 17

float volatge1;
float voltage2;


void setup() {

  	Serial.begin(9600);
    Wire.begin();  // Initialize the i2c bus 1 with pins 18 & 19

	_sensors.setup();
	_airbrakes.setup();

	pinMode(RED, OUTPUT);
	pinMode(BLUE, OUTPUT);
	pinMode(GREEN, OUTPUT);

	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(BACKUP_BATTERY_FLAG, INPUT);
	pinMode(SS_OUT, OUTPUT);

	// Set ADC resolution to 12 bits
	analogReadResolution(ADC_RESOLUTION);

	// Set the SS_OUT pin to HIGH to enable the Vin1 supply
	digitalWrite(SS_OUT, HIGH);
	volatge1 = analogRead(A0) * 3.3 / ADC_MAX;
	voltage2 = analogRead(A1) * 3.3 / ADC_MAX;
}

void loop() {

	

  	if(millis() - _interval > INTERVAL ){

	  	_sensors.loop();
		//_airbrakes.loop();
      	_interval = millis();
  	}

	// Check the battery voltage every 10ms 
	// If the voltage is below the threshold, switch the supply to the backup battery
	  if(millis() - _interval2 > 10 ){

		_interval2 = millis();

		volatge1 = analogRead(A0) * 3.3 / ADC_MAX;
		voltage2 = analogRead(A1) * 3.3 / ADC_MAX; // Dont need it for the switch

		if (volatge1 < VOLTAGE_THRESHOLD && digitalRead(BACKUP_BATTERY_FLAG) == HIGH) {
			// Switch the supply to the backup battery
			digitalWrite(SS_OUT, LOW);
		}

		if(digitalRead(BACKUP_BATTERY_FLAG) == LOW){
			// Switch the RED LED on
			digitalWrite(RED, HIGH);
		}else {
			// Switch the RED LED off
			digitalWrite(RED, LOW);
		}

	}

}

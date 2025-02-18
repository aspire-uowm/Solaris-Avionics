#include <Arduino.h>
#include <Wire.h>
//#include "GPS\GPSsens.h"
#include "PITOT\MPXV7002DP.h"
#include "ICM20948\ICM20948SENSOR.h"
#include "BMP\BMPSensor.h"
#include "Sensors.h"
#include "DHT22\DHT22Sensor.h"
#include "Airbrakes.h"
#include"Servo\PWMServo.h"

#define DHT22_PIN 5
#define SERVO_PIN 4 // TVCX in the PCB schematics
#define INTERVAL 1000


ICM20948Sensor _icm1(&Wire);
BMPSensor _bmp1(0x77, &Wire);
BMPSensor _bmp2(0x76, &Wire);
DHTSensor _dht(DHT22_PIN);

Sensors _sensors(&_icm1, &_bmp1, &_bmp2, &_dht);


uint32_t _interval = 0;

Airbrakes _airbrakes(SERVO_PIN, &_sensors);  // Servo connected to pin 17

void setup() {

  	Serial.begin(9600);
    Wire.begin();  // Initialize the i2c bus 1 with pins 18 & 19

	_sensors.setup();
	_airbrakes.setup();

}

void loop() {

  	if(millis() - _interval > INTERVAL ){

	  	//_sensors.loop();
		_airbrakes.loop();
      	_interval = millis();
  	}

}

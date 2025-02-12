#include <Arduino.h>
#include <Wire.h>
//#include "GPS\GPSsens.h"
#include "PITOT\MPXV7002DP.h"
#include "ICM20948\ICM20948SENSOR.h"
#include "BMP\BMPSensor.h"
#include "Sensors.h"


#define INTERVAL 1000


ICM20948Sensor _icm1(&Wire);
BMPSensor _bmp1(0x77, &Wire);
BMPSensor _bmp2(0x76, &Wire);

Sensors _sensors(&_icm1, &_bmp1, &_bmp2);


uint32_t _interval = 0;


void setup() {

  	Serial.begin(9600);
    Wire.begin();  // Initialize the i2c bus 1 with pins 18 & 19

	_sensors.setup();

}

void loop() {

  	if(millis() - _interval > INTERVAL ){

	  	_sensors.loop();
		
      	_interval = millis();
  	}

}

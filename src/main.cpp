#include <Arduino.h>
#include <Wire.h>
#include "GPS\GPSsens.h"
#include "PITOT\MPXV7002DP.h"
#include "ICM20948/ICM20948SENSOR.h"
#include "BMP\BMPSensor.h"


#define GPS_TX 1
#define GPS_RX 3 
#define PRESSURE_SENSOR_PIN 34
#define INTERVAL 1000 // 1 sec
#define airDensity 1.204



//GPSsens gps(GPS_RX, GPS_TX);

ICM20948Sensor _icm1(&Wire);
ICM20948Sensor _icm2(&Wire1);

BMPSensor _bmp1(&Wire);
BMPSensor _bmp2(&Wire1);

uint32_t _interval = 0;

//PressureSensor pressureSensor(PRESSURE_SENSOR_PIN, airDensity);

void setup() {

  	Serial.begin(9600);
    Wire.begin();  // Initialize the i2c bus 1 with pins 18 & 19
    Wire1.begin(); // Initialize the i2c bus 2 with pins 17 & 16

    _bmp1.setup();
	_bmp2.setup();

	_icm1.setup();
	_icm2.setup();



}

void loop() {

  	if(millis() - _interval > INTERVAL ){
      	_icm1.loop();
	  	_icm2.loop();

	  	_bmp1.loop();
	  	_bmp2.loop();
      	_interval = millis();
  	}

}

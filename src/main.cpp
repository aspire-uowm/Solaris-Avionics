#include <Arduino.h>
#include "Adafruit_ultimate_GPS.h"
#include "MPXV7002DP.h"
#include "MPU6050SENSOR.h"


#define GPS_TX 1
#define GPS_RX 3 
#define PRESSURE_SENSOR_PIN 34
#define INTERVAL 1000

//GPSsens gps(GPS_RX, GPS_TX);

MPU6050Sensor mpu;

uint32_t _interval = 0;

Pressure pressureSensor;
int airspeed = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mpu.setup();
  //gps.begin(); 
  pressureSensor.begin(9600);
  pressureSensor.Init(PRESSURE_SENSOR_PIN); 
}

void loop() {

  if(millis() - _interval > INTERVAL ){
      mpu.loop();
      pressureSensor.loop();
      _interval = millis();
  }

 /* 
  if (gps.readData()) {
    gps.printData();
  }
  */
}

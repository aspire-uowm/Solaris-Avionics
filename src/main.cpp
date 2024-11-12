#include <Arduino.h>
#include "Adafruit_ultimate_GPS.h"
#include "MPXV7002DP.h"
#include "MPU6050SENSOR.h"


#define GPS_TX 1
#define GPS_RX 3 
#define PRESSURE_SENSOR_PIN 34
#define INTERVAL 1000
#define airDensity 1.204

//GPSsens gps(GPS_RX, GPS_TX);

MPU6050Sensor mpu;

uint32_t _interval = 0;

PressureSensor pressureSensor(PRESSURE_SENSOR_PIN, airDensity);

void setup() {
  Serial.begin(9600);
  mpu.setup();
  //gps.begin(); 
  pressureSensor.begin();
}

void loop() {

  if(millis() - _interval > INTERVAL ){
      mpu.loop();
      pressureSensor.airspeed();
      _interval = millis();
  }

 /* 
  if (gps.readData()) {
    gps.printData();
  }
  */
}

#include "Sensors.h"

// 
Sensors::Sensors(ICM20948Sensor* icm, BMPSensor* bmp1, BMPSensor* bmp2, DHTSensor* dht, PressureSensor* pitot): _icm1(icm), 
_bmp1(bmp1), _bmp2(bmp2), _dht(dht), _pitot(pitot) { }

Sensors::~Sensors() { }

void Sensors::setup() {
    Serial.println("Setting up sensors...");
    _icm1->setup();
    _bmp1->setup();
    _bmp2->setup();
    _dht->setup();
    _pitot->setup();
    Serial.println("Sensors setup complete.");
}

void Sensors::loop() {
    _icm1->loop();
    _bmp1->loop();
    _bmp2->loop();
    _dht->loop();
    _pitot->loop();

    update_avg_sensor_data();
}

void Sensors::update_avg_sensor_data() {
    _avg_altitude = (_bmp1->getAltitute() + _bmp2->getAltitute()) / 2.0;
    _avg_temperature = _dht->getTemperature();
    _avg_pressure = (_bmp1->getPressure() + _bmp2->getPressure()) / 2.0;
    _avg_humidity = _dht->getHumidity();
    _icm1->getEulerAngles(_avg_roll, _avg_pitch, _avg_yaw);
    _velocity = _pitot->getSpeed();

    // Serial.print("Average Altitude: "); Serial.print(_avg_altitude); Serial.println(" m");
    // Serial.print("Average Temperature: "); Serial.print(_avg_temperature); Serial.println(" C");
    // Serial.print("Average Pressure: "); Serial.print(_avg_pressure); Serial.println(" Pa");
    // Serial.print("Average Humidity: "); Serial.print(_avg_humidity); Serial.println(" %");
    // Serial.print("Average Roll: "); Serial.print(_avg_roll); Serial.println(" degrees");
    // Serial.print("Average Pitch: "); Serial.print(_avg_pitch); Serial.println(" degrees");
    // Serial.print("Average Yaw: "); Serial.print(_avg_yaw); Serial.println(" degrees");
    Serial.print("Average Velocity: "); Serial.print(_velocity); Serial.println(" m/s");
    Serial.println("===============================================");

}

float Sensors::get_avg_altitude() { return _avg_altitude; }
float Sensors::get_avg_temperature() { return _avg_temperature; }
float Sensors::get_avg_pressure() { return _avg_pressure; }
float Sensors::get_avg_humidity() { return _avg_humidity; }
float Sensors::get_velocity() { return _velocity; }
float Sensors::get_avg_roll() { return _avg_roll; }
float Sensors::get_avg_pitch() { return _avg_pitch; }
float Sensors::get_avg_yaw() { return _avg_yaw; }


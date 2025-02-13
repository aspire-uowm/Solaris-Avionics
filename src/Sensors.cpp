#include "Sensors.h"


Sensors::Sensors(ICM20948Sensor* icm, BMPSensor* bmp1, BMPSensor* bmp2, DHTSensor* dht): _icm1(icm), 
_bmp1(bmp1), _bmp2(bmp2), _dht(dht) {
    

}

Sensors::~Sensors() { }

void Sensors::setup() {
    Serial.println("Setting up sensors...");
    _icm1->setup();
    _bmp1->setup();
    _bmp2->setup();
    _dht->setup();
    Serial.println("Sensors setup complete.");
}

void Sensors::loop() {
    _icm1->loop();
    _bmp1->loop();
    _bmp2->loop();
    _dht->loop();

    update_avg_sensor_data();
}

void Sensors::update_avg_sensor_data() {
    _avg_altitude = (_bmp1->getAltitute() + _bmp2->getAltitute()) / 2.0;
    _avg_temperature = _dht->getTemperature();
    _avg_pressure = (_bmp1->getPressure() + _bmp2->getPressure()) / 2.0;
    _avg_humidity = _dht->getHumidity();
    _avg_acc_x = _icm1->getAccX();
    _avg_acc_y = _icm1->getAccY();
    _avg_acc_z = _icm1->getAccZ();
    _avg_angle_x = _icm1->getAngleX();
    _avg_angle_y = _icm1->getAngleY();
    _avg_angle_z = _icm1->getAngleZ();
    _avg_velocity_x = _icm1->getVelocityX();
    _avg_velocity_y = _icm1->getVelocityY();
    _avg_velocity_z = _icm1->getVelocityZ();



    Serial.print("Average Altitude: "); Serial.print(_avg_altitude); Serial.println(" m");
    // Serial.print("Altitute bmp1: "); Serial.println(_bmp1->getAltitute());
    // Serial.print("Altitute bmp2: "); Serial.println(_bmp2->getAltitute());
    Serial.print("Average Temperature: "); Serial.print(_avg_temperature); Serial.println(" C");
    Serial.print("Average Pressure: "); Serial.print(_avg_pressure); Serial.println(" Pa");
    Serial.print("Average Humidity: "); Serial.print(_avg_humidity); Serial.println(" %");
    Serial.println("===============================================");
    Serial.print("Average Acc X: "); Serial.println(_avg_acc_x);
    Serial.print("Average Acc Y: "); Serial.println(_avg_acc_y);
    Serial.print("Average Acc Z: "); Serial.println(_avg_acc_z);
    Serial.println("===============================================");
    Serial.print("Average Angle X: "); Serial.println(_avg_angle_x);
    Serial.print("Average Angle Y: "); Serial.println(_avg_angle_y);
    Serial.print("Average Angle Z: "); Serial.println(_avg_angle_z);
    Serial.println("===============================================");
    Serial.print("Average Velocity X: "); Serial.println(_avg_velocity_x);
    Serial.print("Average Velocity Y: "); Serial.println(_avg_velocity_y);
    Serial.print("Average Velocity Z: "); Serial.println(_avg_velocity_z);
    Serial.println("===============================================");
}

float Sensors::get_avg_altitude() { return _avg_altitude; }
float Sensors::get_avg_temperature() { return _avg_temperature; }
float Sensors::get_avg_pressure() { return _avg_pressure; }
float Sensors::get_avg_humidity() { return _avg_humidity; }

float Sensors::get_avg_acc_x() { return _avg_acc_x; }
float Sensors::get_avg_acc_y() { return _avg_acc_y; }
float Sensors::get_avg_acc_z() { return _avg_acc_z; }

float Sensors::get_avg_angle_x() { return _avg_angle_x; }
float Sensors::get_avg_angle_y() { return _avg_angle_y; }
float Sensors::get_avg_angle_z() { return _avg_angle_z; }

float Sensors::get_avg_velocity_x() { return _avg_velocity_x; }
float Sensors::get_avg_velocity_y() { return _avg_velocity_y; }
float Sensors::get_avg_velocity_z() { return _avg_velocity_z; }

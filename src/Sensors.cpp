#include "Sensors.h"


Sensors::Sensors(ICM20948Sensor* icm, BMPSensor* bmp1, BMPSensor* bmp2): _icm1(icm), _bmp1(bmp1), _bmp2(bmp2) {

}

Sensors::~Sensors() { }

void Sensors::setup() {
    Serial.println("Setting up sensors...");
    _icm1->setup();
    _bmp1->setup();
    _bmp2->setup();
    Serial.println("Sensors setup complete.");
}

void Sensors::loop() {
    _icm1->loop();
    _bmp1->loop();
    _bmp2->loop();

    update_avg_sensor_data();
}

void Sensors::update_avg_sensor_data() {
    _avg_altitude = (_bmp1->getAltitute() + _bmp2->getAltitute()) / 2.0;
    _avg_temperature = (_bmp1->getTemperature() + _bmp2->getTemperature()) / 2.0;
    _avg_pressure = (_bmp1->getPressure() + _bmp2->getPressure()) / 2.0;
    // _avg_acc_x = (_icm1->getAccX() + _icm1->getAccX()) / 2.0;
    // _avg_acc_y = (_icm1->getAccY() + _icm1->getAccY()) / 2.0;
    // _avg_acc_z = (_icm1->getAccZ() + _icm1->getAccZ()) / 2.0;
    // _avg_angle_x = (_icm1->getAngleX() + _icm1->getAngleX()) / 2.0;
    // _avg_angle_y = (_icm1->getAngleY() + _icm1->getAngleY()) / 2.0;
    // _avg_angle_z = (_icm1->getAngleZ() + _icm1->getAngleZ()) / 2.0;

    _avg_acc_x = _icm1->getAccX();
    _avg_acc_y = _icm1->getAccY();
    _avg_acc_z = _icm1->getAccZ();
    _avg_angle_x = _icm1->getAngleX();
    _avg_angle_y = _icm1->getAngleY();
    _avg_angle_z = _icm1->getAngleZ();


    Serial.print("Average Altitude: "); Serial.println(_avg_altitude);
    // Serial.print("Altitute bmp1: "); Serial.println(_bmp1->getAltitute());
    // Serial.print("Altitute bmp2: "); Serial.println(_bmp2->getAltitute());
    Serial.print("Average Temperature: "); Serial.println(_avg_temperature);
    Serial.print("Average Pressure: "); Serial.println(_avg_pressure);
    Serial.print("Average Acc X: "); Serial.println(_avg_acc_x);
    Serial.print("Average Acc Y: "); Serial.println(_avg_acc_y);
    Serial.print("Average Acc Z: "); Serial.println(_avg_acc_z);
    Serial.print("Average Angle X: "); Serial.println(_avg_angle_x);
    Serial.print("Average Angle Y: "); Serial.println(_avg_angle_y);
    Serial.print("Average Angle Z: "); Serial.println(_avg_angle_z);
}

float Sensors::get_avg_altitude() { return _avg_altitude; }
float Sensors::get_avg_temperature() { return _avg_temperature; }
float Sensors::get_avg_pressure() { return _avg_pressure; }
float Sensors::get_avg_acc_x() { return _avg_acc_x; }
float Sensors::get_avg_acc_y() { return _avg_acc_y; }
float Sensors::get_avg_acc_z() { return _avg_acc_z; }
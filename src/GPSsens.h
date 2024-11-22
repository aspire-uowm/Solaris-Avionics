#ifndef GPSSENS_H
#define GPSSENS_H

#include <Adafruit_GPS.h>
#include <Arduino.h> // Required for PlatformIO projects

class GPSsens {
private:
    HardwareSerial gpsSerial;
    Adafruit_GPS GPS;
    int rxPin;
    int txPin;
    long baudRate;

public:
    GPSsens(int rxPin, int txPin, long baudRate = 9600);
    void begin();
    bool readData();
    float getLat() const;
    float getLong() const;
    float getAlt() const;
    float getSpeed() const;
    void printData();
    void setUpdateRate(int hz);


};
#endif // GPSSENS_H

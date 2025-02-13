#ifndef DHT22Sensor_H
#define DHT22Sensor_H

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>



#define DHTTYPE DHT22

class DHTSensor
{
private:

    DHT _dht;

    float _temperature;
    float _humidity;

    float _temp_error;
    float _humidity_error;
    
public:
    // Constructor
    DHTSensor(int pin);
    // Destructor
    ~DHTSensor();

    // Methods
    void setup();
    void loop();

    // Getters
    float getTemperature();
    float getHumidity();

    // Error correction
    void calculateDHTErrors();
};


#endif
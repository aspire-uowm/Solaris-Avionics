#include "DHT22Sensor.h"


DHTSensor::DHTSensor(int pin) : _dht(pin, DHTTYPE) {}

DHTSensor::~DHTSensor() {}

void DHTSensor::setup() {
    _dht.begin();

    // Check connection
    int retries = 5;
    while (retries-- > 0) {
        if (!isnan(_dht.readTemperature()) && !isnan(_dht.readHumidity())) {
            Serial.println("DHT22 connected successfully!");
            break;
        }
        delay(500); // Wait 500ms before retrying
        Serial.println("DHT22 connection failed. Retrying...");

        if(retries == 0) {
            Serial.println("DHT22 connection failed. Aborting setup.");
        }
    }
    
    Serial.print("Calculating Errors for DHT22... ");
    calculateDHTErrors();   
}


void DHTSensor::loop() {
    static unsigned long lastRead = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastRead >= 100) {  // Read every 100 milliseconds
        lastRead = currentTime;
        _temperature = _dht.readTemperature() - _temp_error;
        _humidity = _dht.readHumidity() - _humidity_error;
    }
}


float DHTSensor::getTemperature() {
    return _temperature;
}

float DHTSensor::getHumidity() {
    return _humidity;
}

void DHTSensor::calculateDHTErrors() {
    const int samplecount = 200;   // Number of samples
    const int interval = 20;       // Sample interval (20ms)
    const unsigned long duration = 4000;  // Total duration (4s)

    float temp_sum = 0.0;
    float humidity_sum = 0.0;

    _temp_error = 0.0;   // Reset errors
    _humidity_error = 0.0;
    int validSamples = 0;

    Serial.println("Place an accurate reference thermometer and hygrometer next to the DHT22.");
    delay(5000);  // Give the user time to prepare

    Serial.println("Measuring reference values...");
    delay(2000);  // Allow the sensor to stabilize

    // Take a single reference measurement
    float reference_temp = _dht.readTemperature();
    float reference_humidity = _dht.readHumidity();

    if (isnan(reference_temp) || isnan(reference_humidity)) {
        Serial.println("Error: Failed to read reference values. Check sensor connection!");
        return;
    }

    Serial.print("Reference Temperature: "); Serial.println(reference_temp);
    Serial.print("Reference Humidity: "); Serial.println(reference_humidity);
    Serial.println("Starting calibration...");

    unsigned long startTime = millis();
    unsigned long lastSampleTime = 0;

    while (millis() - startTime < duration) {
        if (millis() - lastSampleTime >= interval) {  // Sample every 20ms
            lastSampleTime = millis();

            float temp = _dht.readTemperature();
            float humidity = _dht.readHumidity();

            if (!isnan(temp) && !isnan(humidity)) {  // Only count valid readings
                temp_sum += (temp - reference_temp);  // Compute error (bias)
                humidity_sum += (humidity - reference_humidity);
                validSamples++;
            }

            if (validSamples >= samplecount) break;  // Stop if we reach 200 valid samples
        }
    }

    if (validSamples > 0) {  // Avoid division by zero
        _temp_error = temp_sum / validSamples;   // Compute average bias
        _humidity_error = humidity_sum / validSamples;
    }

    Serial.println("Calibration Complete!");
    Serial.print("Temperature Bias: "); Serial.println(_temp_error);
    Serial.print("Humidity Bias: "); Serial.println(_humidity_error);
}



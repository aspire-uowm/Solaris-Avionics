#include <Adafruit_GPS.h>

class GPSsens{
public:
    GPSsens(int rxPin, int txPin, long baudRate = 9600):
        gpsSerial(1), rxPin(rxPin), txPin(txPin), baudRate(baudRate), GPS(&gpsSerial){}

    void begin() {
        //GPS hardware serial begin
        Serial.begin(9600); 
        gpsSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
        GPS.begin(baudRate);
        

        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);//gps output format
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);//update rate of 10hz



    } 

    //Boolean to only process data when there has been a change to the data
    bool readData() {
        char c = GPS.read();
        if (GPS.newNMEAreceived()) {
            if (GPS.parse(GPS.lastNMEA())) {
                return true;
            } else {
                Serial.println("Failed to parse NMEA sentence.");
            }
        }
        return false;
    }

    }
    //Functions that get the GPS data
    float getLat() const { return GPS.latitude; }
    float getLong() const { return GPS.longitude; }
    float getAlt() const { return GPS.altitude; }
    float getSpeed() const { return GPS.speed; }

    void printData() {
        if (!GPS.fix) {
            Serial.println("GPS:Waiting for GPS fix...");
            return;
        }
        Serial.print("GPS:Latitude: "); Serial.println(getLat(), 4);
        Serial.print("GPS:Longitude: "); Serial.println(getLong(), 4);
        Serial.print("GPS:Altitude: "); Serial.println(getAlt());
        Serial.print("GPS:Speed: "); Serial.println(getSpeed());
        Serial.println();
    }

    void setUpdateRate(int hz) {
        if (hz == 1) {
            GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        } else if (hz == 5) {
            GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
        } else if (hz == 10) {
            GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
        } else {
            Serial.println("GPS:Unsupported update rate. Defaulting to 1 Hz.");
            GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        }
    }


private:
    HardwareSerial gpsSerial;
    Adafruit_GPS GPS;
    int rxPin;
    int txPin;
    long baudRate;
};
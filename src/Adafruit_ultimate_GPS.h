#include <Adafruit_GPS.h>

class GPSsens{
public:
    GPSsens(int rxPin, int txPin, long baudRate = 9600):
        gpsSerial(1), rxPin(rxPin), txPin(txPin), baudRate(baudRate), GPS(&gpsSerial){}

    void begin() {
        //GPS hardware serial begin
        Serial.begin(115200); 
        gpsSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
        GPS.begin(baudRate);
        

        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);//gps output format
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);//update rate of 10hz



    } 

    //Boolean to only process data when there has been a change to the data
    bool readData(){
        char c = GPS.read();

        if (GPS.newNMEAreceived()){
            if (GPS.parse(GPS.lastNMEA())){
                return true;
            }
        }
        return false;
    }
    //Functions that get the GPS data
    float getLat() const { return GPS.latitude; }
    float getLong() const { return GPS.longitude; }
    float getAlt() const { return GPS.altitude; }
    float getSpeed() const { return GPS.speed; }

    //GPS serial printing section of the code
    void printData() {
        Serial.print("Latitude: "); Serial.println(getLat(), 4);
        Serial.print("Longitude: "); Serial.println(getLong(), 4);
        Serial.print("Altitude: "); Serial.println(getAlt());
        Serial.print("Speed: "); Serial.println(getSpeed());
        Serial.println();
    }


private:
    HardwareSerial gpsSerial;
    Adafruit_GPS GPS;
    int rxPin;
    int txPin;
    long baudRate;
};
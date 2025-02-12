// #include "GPSsens.h"

// // Constructor
// GPSsens::GPSsens(int rxPin, int txPin, long baudRate)
//     : gpsSerial(1), rxPin(rxPin), txPin(txPin), baudRate(baudRate), GPS(&gpsSerial) {}

// // Initialize the GPS module
// void GPSsens::begin() {
//     Serial.begin(9600);  // Debug Serial Monitor
//     gpsSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
//     GPS.begin(baudRate);

//     GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Set output to GPRMC and GPGGA sentences
//     GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // Set update rate to 10 Hz
//     Serial.println("GPS initialized. Attempting to connect to satellites...");
// }

// // Read data from the GPS module
// bool GPSsens::readData() {
//     while (GPS.available()) {
//         char c = GPS.read(); // Read GPS data
//         if (GPS.newNMEAreceived()) {
//             if (GPS.parse(GPS.lastNMEA())) {
//                 return true;
//             } else {
//                 Serial.println("Failed to parse NMEA sentence.");
//             }
//         }
//     }
//     return false;
// }

// // Get latitude
// float GPSsens::getLat() const {
//     return GPS.latitude;
// }

// // Get longitude
// float GPSsens::getLong() const {
//     return GPS.longitude;
// }

// // Get altitude
// float GPSsens::getAlt() const {
//     return GPS.altitude;
// }

// // Get speed
// float GPSsens::getSpeed() const {
//     return GPS.speed;
// }

// // Print GPS data
// void GPSsens::printData() {
//     static unsigned long lastPrint = 0;
//     unsigned long currentTime = millis();

//     if (!GPS.fix) {
//         if (currentTime - lastPrint >= 2000) { // Print every 2 seconds while connecting
//             Serial.println("GPS: Attempting to acquire satellite fix...");
//             lastPrint = currentTime;
//         }
//         return;
//     }

//     Serial.print("GPS: Latitude: "); Serial.println(getLat(), 4);
//     Serial.print("GPS: Longitude: "); Serial.println(getLong(), 4);
//     Serial.print("GPS: Altitude: "); Serial.println(getAlt());
//     Serial.print("GPS: Speed: "); Serial.println(getSpeed());
//     Serial.println();
// }

// // Set the update rate of the GPS module
// void GPSsens::setUpdateRate(int hz) {
//     if (hz == 1) {
//         GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//     } else if (hz == 5) {
//         GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
//     } else if (hz == 10) {
//         GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
//     } else {
//         Serial.println("GPS: Unsupported update rate. Defaulting to 1 Hz.");
//         GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//     }
// }

// // Check if GPS is attempting to connect to satellites
// bool GPSsens::isConnecting() const {
//     return !GPS.fix;
// }

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>


#define V_REF 5.0
#define VOLTAGE_MULTIPLIER 11.0
#define CURRENT_MULTIPLIER 100.0
#define CURRENT_OFFSET 2.5
#define ADC_STATES 1024.0
#define VOLTAGE_PIN A2
#define CURRENT_PIN A5
#define GPSECHO false
#define LOGGING_DELAY 1000


SoftwareSerial gpsSerial(8, 7);
Adafruit_GPS GPS(&gpsSerial);
uint32_t timer = millis();
File logFile;


void setup() {
    // connect at 115200 so we can read the GPS fast enough
    Serial.begin(115200);
    //while (!Serial); // wait for leo to be ready
    delay(5000);

    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(CURRENT_PIN, INPUT);

    // SD card: pin 10 for chip select, 11 for MOSI, 12 for MISO and 13 for SCK
    //if (!SD.begin(10)) {
    if (!SD.begin(10, 11, 12, 13)) {
        Serial.println("SD initialization failed!");
        while(1);
    }
    logFile = SD.open("data.csv", FILE_WRITE);
    if (!logFile) {
        Serial.println("Failed to open log file!");
        while(1);
    }
    
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
    GPS.begin(9600);
    
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz    
    delay(1000);
}

void log_data(float latitude, float longitude, float speed, float angle, float altitude, float voltage, float current) {
    unsigned long time = millis();
    if (logFile) {
        logFile.print(time);
        logFile.print(", ");
        logFile.print(latitude, 8);
        logFile.print(", ");
        logFile.print(longitude, 8);
        logFile.print(", ");
        logFile.print(speed, 4);
        logFile.print(", ");
        logFile.print(angle, 4);
        logFile.print(", ");
        logFile.print(altitude, 4);
        logFile.print(", ");
        logFile.print(voltage, 4);
        logFile.print(", ");
        logFile.println(current, 4);
        logFile.flush();
    }

    /*Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

    Serial.print(time);
    Serial.print(", ");
    Serial.print(latitude);
    Serial.print(", ");
    Serial.print(longitude);
    Serial.print(", ");
    Serial.print(speed);
    Serial.print(", ");
    Serial.print(altitude);
    Serial.print(", ");
    Serial.print(voltage);
    Serial.print(", ");
    Serial.println(current);*/
}

void loop() {
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if ((c) && (GPSECHO))
      Serial.write(c); 
    
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {    
        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
          return;  // we can fail to parse a sentence in which case we should just wait for another
    }
  
    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();
  
    if (millis() - timer > LOGGING_DELAY) {
        timer = millis(); // reset the timer
        float voltage = 0.0;  // calculated voltage
        float current = 0.0;  // calculated current
        float latitude = 0.0;
        float longitude = 0.0;
        float speed = 0.0;  // in knots
        float angle = 0.0;
        float altitude = 0.0;  // in meters
        if (GPS.fix) {
            latitude = GPS.latitude / 100.0;
            longitude = GPS.longitude / 100.0;
            speed = GPS.speed;
            angle = GPS.angle;
            altitude = GPS.altitude;
        }

        int voltsValue = analogRead(VOLTAGE_PIN);
        int currentValue = analogRead(CURRENT_PIN);
        
        // calculate the voltage
        voltage = ((float)voltsValue * V_REF) / ADC_STATES * VOLTAGE_MULTIPLIER;
        // calculate the current
        current = (((float)currentValue * V_REF) / ADC_STATES - CURRENT_OFFSET) * CURRENT_MULTIPLIER;

        log_data(latitude, longitude, speed, angle, altitude, voltage, current);
        // TODO: get pedal positions
    }
}


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
#define STARTING_DELAY 5000
#define LOGGING_DELAY 100


Adafruit_GPS *gps;
uint32_t timer = millis();
File logFile;
char outputBuffer[80];


void setup() {
    SoftwareSerial gpsSerial(8, 7);
    gps = new Adafruit_GPS(&gpsSerial);
    
    // connect at 115200 so we can read the GPS fast enough
    Serial.begin(115200);
    delay(STARTING_DELAY);

    // Configure pins for reading
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(CURRENT_PIN, INPUT);

    // SD card: pin 10 for chip select, 11 for MOSI, 12 for MISO and 13 for SCK
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
    gps->begin(9600);
    
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time    
    //gps->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
    gps->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // Turn on only the "minimum recommended" data
    
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz update rate
    //gps->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    gps->sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 10 Hz update rate
    
    delay(1000);
}

void loop() {
    // Read from GPS
    char c = gps->read();
    
    // If a GPS sentence is received, we can check the checksum and parse it...
    if (gps->newNMEAreceived()) {    
        if (!gps->parse(gps->lastNMEA()))   // this also sets the newNMEAreceived() flag to false
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
        if (gps->fix) {
            latitude = gps->latitude / 100.0;
            longitude = gps->longitude / 100.0;
            speed = gps->speed;
            angle = gps->angle;
            altitude = gps->altitude;
        }

        int voltsValue = analogRead(VOLTAGE_PIN);
        int currentValue = analogRead(CURRENT_PIN);
        
        // calculate the voltage
        voltage = ((float)voltsValue * V_REF) / ADC_STATES * VOLTAGE_MULTIPLIER;
        // calculate the current
        current = (((float)currentValue * V_REF) / ADC_STATES - CURRENT_OFFSET) * CURRENT_MULTIPLIER;

        //log_data(latitude, longitude, speed, angle, altitude, voltage, current);
        print_data(latitude, longitude, speed, angle, altitude, voltage, current);
        //print_gps();
    }
}

void log_data(float latitude, float longitude, float speed, float angle, float altitude, float voltage, float current) {
    if (logFile) {
        unsigned long time = millis();
        sprintf(outputBuffer, "%10d, %12.8f, %12.8f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f", time, latitude, longitude, speed, angle, altitude, voltage, current);
        logFile.println(outputBuffer);
    }
}

void flush_log() {
    if (logFile) {
        logFile.flush();
    }
}

void print_data(float latitude, float longitude, float speed, float angle, float altitude, float voltage, float current) {
    unsigned long time = millis();
    sprintf(outputBuffer, "%10d, %12.8f, %12.8f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f", time, latitude, longitude, speed, angle, altitude, voltage, current);
    Serial.println(outputBuffer);
}

void print_gps() {
    Serial.print(gps->hour, DEC); Serial.print(':');
    Serial.print(gps->minute, DEC); Serial.print(':');
    Serial.print(gps->seconds, DEC); Serial.print('.');
    Serial.println(gps->milliseconds);
    Serial.print("Date: ");
    Serial.print(gps->day, DEC); Serial.print('/');
    Serial.print(gps->month, DEC); Serial.print("/20");
    Serial.println(gps->year, DEC);
    Serial.print("Fix: "); Serial.print((int)gps->fix);
    Serial.print(" quality: "); Serial.println((int)gps->fixquality); 
    Serial.print("Satellites: "); Serial.println((int)gps->satellites);
}


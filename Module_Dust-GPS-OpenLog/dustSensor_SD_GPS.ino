/*
 Module using an Arduino Fio, a Sharp Optical Dust Sensor GP2Y1010AU0F,
 an Adafruit Ultimate GPS Breakout v3 and an OpenLog data logger.
 
 Blog: http://arduinodev.woofex.net/2012/12/16/sharp-dust-sensor-with-adafruit-gps-v3/
 Code: https://github.com/Trefex/arduino-airquality/

 For Pin connections, please check the Blog or the github project page
 Authors: Cyrille Médard de Chardon (serialC), Christophe Trefois (Trefex)
 Changelog:
   2012-Dec-16: First version working. Need to clean-up code

 This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License. 
 To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ or send a letter 
 to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA. 
 */
 
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Arduino.h>
 
#define measurePin 6
#define ledPower 12

#define USE_OPENLOG // disable to remove logging
#define USE_GPS // disable to remove GPS
//#define USE_BARO // disable to remove Barometric sensor
#define DEBUG_ON // enable to output debugging information on normal Serial
#define GPSECHO false // put to true if you want to see raw GPS data flowing

#ifdef USE_BARO
  #include <Adafruit_BMP085.h>
#endif
#ifdef USE_GPS
  #include <Adafruit_GPS.h>
#endif 

#define OPENLOG_RST_PIN 6
#define OPENLOG_TX_PIN 7
#define OPENLOG_RX_PIN 8
#define ADAGPS_RX_PIN 2 // GPS RX to Pin ADAGPS_RX_PIN
#define ADAGPS_TX_PIN 3 // GPS TX to ADAGPS_TX_PIN
#define ADAGPS_RESET 10 // Pull Low to switch off the module

#ifdef USE_OPENLOG
  SoftwareSerial OpenLog(OPENLOG_RX_PIN, OPENLOG_TX_PIN);
#endif

#ifdef USE_BARO
  Adafruit_BMP085 BMP;
#endif

#ifdef USE_GPS
  // Create Soft Serial GPS handle
  SoftwareSerial GPSSerial(ADAGPS_TX_PIN, ADAGPS_RX_PIN);
  // Create GPS handle using GPS Soft Serial
  Adafruit_GPS GPS(&GPSSerial);
#endif

#ifdef USE_OPENLOG
  //static const int resetOpenLog = OPENLOG_RST_PIN;
  boolean openlog_ready = false;
#endif

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
boolean gpsReady = false;

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

uint32_t timer = millis();

void setup(){
  #ifdef DEBUG_ON
    Serial.begin(115200);
  #endif
  
  pinMode(ledPower,OUTPUT);
  pinMode(ADAGPS_RESET, OUTPUT);
  
  digitalWrite(ADAGPS_RESET, LOW);
  delay(2000);
  digitalWrite(ADAGPS_RESET, HIGH);
  
  #ifdef USE_GPS
    GPS.begin(9600);
    delay(40);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  #endif
  
  #ifdef USE_OPENLOG
    // Setup OpenLog
    OpenLog.begin(9600);
    pinMode(OPENLOG_RST_PIN, OUTPUT);  
  #endif
  
  #ifdef USE_BARO
    BMP.begin();
  #endif

  delay(1000);
  #ifdef USE_GPS
    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);
  #endif
}

void loop(){
  delay(2000);
  #ifdef DEBUG_ON
    Serial.println("\nBeing of loop");
  #endif

  // Take measurement every two seconds
  // if (millis() - timer > 2000) { 
    #ifdef USE_OPENLOG
      OpenLog.listen();
      delay(20);
    #endif
 
    digitalWrite(ledPower,LOW); // power on the LED
    delayMicroseconds(samplingTime);
  
    voMeasured = analogRead(measurePin); // read the dust value
    
    delayMicroseconds(deltaTime);
    digitalWrite(ledPower,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);
  
    // 0 - 3.3V mapped to 0 - 1023 integer values
    // recover voltage
    calcVoltage = voMeasured * (3.3 / 1024); 
    
    // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
    // Chris Nafis (c) 2012
    dustDensity = (0.17 * calcVoltage - 0.1)*1000; 
    
    if(dustDensity < 0) {
      dustDensity = 0;
    }
    
    // Begin new line
    #ifdef USE_OPENLOG

    #endif
    
    #ifdef USE_GPS // If GPS is on, print all sensor values, else only Dust
      if(GPS.fix) {
        #ifdef USE_OPENLOG
          OpenLog.print("<< ");
          OpenLog.print(GPS.day, DEC); OpenLog.print("/");
          OpenLog.print(GPS.month, DEC); OpenLog.print("/20");
          OpenLog.print(GPS.year, DEC); OpenLog.print(";");
          OpenLog.print(GPS.hour, DEC); OpenLog.print(":");
          delay(15);
          OpenLog.print(GPS.minute, DEC); OpenLog.print(":");
          OpenLog.print(GPS.seconds, DEC); OpenLog.print(";");
          OpenLog.print(GPS.latitude, 4); OpenLog.print(GPS.lat); OpenLog.print(";"); 
          OpenLog.print(GPS.longitude, 4); OpenLog.print(GPS.lon); OpenLog.print(";");
          // Dust Values
          OpenLog.print(dustDensity); OpenLog.print(";");
          delay(15);
          #ifdef USE_BARO
            OpenLog.print(BMP.readTemperature()); OpenLog.print(";");
            OpenLog.println(BMP.readPressure()); 
          #endif
          delay(15);
        #endif
      }
    #else
      #ifdef USE_OPENLOG
        OpenLog.print("<< ");
        OpenLog.print(dustDensity); OpenLog.print(";");
      #endif
    #endif 
    
    #ifdef USE_BARO
      #ifdef DEBUG_ON
        Serial.print("\nTemperature: "); Serial.print(BMP.readTemperature()); Serial.println(" *C");
        Serial.print("Pressure: "); Serial.print(BMP.readPressure()); Serial.println(" Pa");
        Serial.print("Altitude: "); Serial.print(BMP.readAltitude()); Serial.println(" meters");
      #endif  
      #ifndef USE_GPS // Print BMP alone when no GPS is present
        #ifdef USE_OPENLOG
          OpenLog.print(BMP.readTemperature()); OpenLog.print(";");
          OpenLog.println(BMP.readPressure()); 
          delay(15);
        #endif
      #endif
    #endif
    
    #ifdef DEBUG_ON 
      Serial.print("Dust Density [ug/m3]: ");  Serial.println(dustDensity);
    #endif
 // }
  
  #ifdef USE_GPS
    GPSSerial.listen();
    //delay(1000);
  #endif

  unsigned long start = millis(); // begin listening to the GPS
  // Listen to GPS for two seconds or until fix has been reached, whichever comes first
  #ifdef USE_GPS
    while (start + 2000 > millis()) {
      if (! usingInterrupt) {
          // read data from the GPS in the 'main loop'
          char c = GPS.read();
          // if you want to debug, this is a good time to do it!
          if (GPSECHO)
            if (c) UDR0 = c;
            // writing direct to UDR0 is much much faster than Serial.print 
            // but only one character can be written at a time. 
      }
      
      // if a sentence is received, we can check the checksum, parse it...
      if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences! 
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
        // Serial.println("New GPS Sentence received");
        //gpsReady = true;
        //break;
        if (GPS.parse(GPS.lastNMEA())){   // this also sets the newNMEAreceived() flag to false
          if (GPS.fix) {
            #ifdef DEBUG_ON 
                Serial.print("\nTime: ");
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
                Serial.print("Location: ");
                Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
                Serial.print(", "); 
                Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
                
                Serial.print("Speed (knots): "); Serial.println(GPS.speed);
                Serial.print("Angle: "); Serial.println(GPS.angle);
                Serial.print("Altitude: "); Serial.println(GPS.altitude);
                Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
                break;
            #endif
          }
        }
      }
    }
  #endif
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
  //clearSerial();
  //delay(400);
}

void clearSerial() {
  #ifdef DEBUG_ON
    while (Serial.read() >= 0) {;}
  #endif
  #ifdef USE_GPS
    while(GPSSerial.read() >= 0) {;}
  #endif
  //while(OpenLog.read() >= 0){;}
}



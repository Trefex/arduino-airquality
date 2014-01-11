/*
 Module using an Arduino Fio, a Sharp Optical Dust Sensor GP2Y1010AU0F,
 an Adafruit Ultimate GPS Breakout 3v3 and an OpenLog data logger.
 
 Blog: TO COME
 Code: https://github.com/Trefex/arduino-airquality/

 For Pin connections, please check the Blog or the github project page
 Authors: Cyrille Médard de Chardon (serialC), Christophe Trefois (Trefex)
 Changelog:
   2014-Dec-11: Added CO code through library addition
   2012-Dec-16: First version working. Need to clean-up code
   2013-Dec-22: Added CO Sensor

 This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License. 
 To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ or send a letter 
 to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA. 
 */
 
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Arduino.h>

// Allow for basic C++ functions
#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <utility.h>
#include <vector>

using namespace std;

#define USE_OPENLOG // disable to remove logging
#define USE_GPS // disable to remove GPS
#define USE_BARO // disable to remove Barometric sensor
#define DEBUG_ON // enable to output debugging information on normal Serial
#define GPSECHO true // put to true if you want to see raw GPS data flowing
#define USE_HT // comment to remove Humidity and temperature sensor
#define USE_CO // disable to remove CO sensor

// Check which sensors are being used based on defines above
#ifdef USE_BARO
  #include "Adafruit_BMP085.h"
#endif

#ifdef USE_GPS
  #include "Adafruit_GPS.h"
#endif

#ifdef USE_HT
  #include "SHT1x.h"
#endif

#ifdef USE_CO
  #include "COS_MQ7.h"
#endif

// Pin definitions
#define HT_DATA_PIN 11  // Humidity/Temp sensor data
#define HT_SCK_PIN 13  // Humidity/Temp sensor serial clock

#define DUST_RX_PIN 6  // Dust sensor (Analog 6)
#define DUST_LED_PIN 12  // Dust sensor led

#define OPENLOG_RST_PIN 6  // Openlogger (Digital 6)
#define OPENLOG_TX_PIN 7  // Openlogger
#define OPENLOG_RX_PIN 8  // Openlogger

#define ADAGPS_RX_PIN 2 // GPS RX to Pin ADAGPS_RX_PIN
#define ADAGPS_TX_PIN 3 // GPS TX to ADAGPS_TX_PIN
#define ADAGPS_RESET 10 // Pull Low to switch off the module

#define ACTIVE_MONOX_LED_PIN 9 // CO LED Pin
#define ACTIVE_MONOX_PIN 5 // CO Switch Pin
#define READ_MONOX_PIN A2 // CO Read Pin
#define READ_COPSV_PIN A3 // CO Power Supply Voltage


#ifdef USE_CO
  COS_MQ7 MQ7(ACTIVE_MONOX_LED_PIN, ACTIVE_MONOX_PIN, READ_MONOX_PIN, READ_COPSV_PIN);
#endif

// Constructor declarations
#ifdef USE_HT
  SHT1x th_sensor(HT_DATA_PIN, HT_SCK_PIN);
#endif

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

// variable initializations
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
float temp_c = -99;
float humid = -99;

int co_voltage = 0;

uint32_t timer = millis();

void setup(){
  #ifdef DEBUG_ON
    // 115200 baud rate for connection
    Serial.begin(115200);
  #endif
  
  // define pins as output mode
  pinMode(DUST_LED_PIN, OUTPUT);
  //pinMode(ADAGPS_RESET, OUTPUT);
  
  // reset the GPS, but we the cable is not connected
  //digitalWrite(ADAGPS_RESET, LOW);
  //delay(2000);
  //digitalWrite(ADAGPS_RESET, HIGH);
  
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
    Serial.println("\nBegining of loop");
  #endif
  
  

  // Take measurement every two seconds
  // if (millis() - timer > 2000) { 
    #ifdef USE_OPENLOG
      // listen for data
      OpenLog.listen();
      delay(20);
    #endif
 
    digitalWrite(DUST_LED_PIN,LOW); // power on the LED
    delayMicroseconds(samplingTime);
  
    voMeasured = analogRead(DUST_RX_PIN); // read the dust value
    
    delayMicroseconds(deltaTime);
    digitalWrite(DUST_LED_PIN,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);
  
    // VoMeasured returns a value between 0-1023 relative to 3.3V 
    // We convert this value to a voltage value used for dust density
    calcVoltage = 3.3 * (voMeasured / 1023);
    // Note that we use a step up voltage regulator to get 5V powering the dust sensor
    
    // Convert voltage to density
    // Convert voltage to dust density (ug/m3) based on
    // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
    // Chris Nafis (c) 2012
    dustDensity = (0.17 * calcVoltage - 0.1)*1000;
    
    //if(dustDensity < 0) {
      //dustDensity = 0;
    //}


    // Get temperature
    #ifdef USE_HT
      // Read values from the sensor, humidity first
      humid = th_sensor.readHumidity();
      temp_c = th_sensor.retrieveTemperatureC();
    #endif
    
    // Cycle Power of CO Sensor
    #ifdef USE_CO
      co_voltage = -1;
      MQ7.Power_cycle();
      if(MQ7.Get_state() == 4) {
       co_voltage = MQ7.Get_CO_reading();
      }
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
            OpenLog.print(BMP.readPressure()); OpenLog.print(";");
            delay(15);
          #endif
          
          #ifdef USE_HT
            OpenLog.print(temp_c); OpenLog.print(";");
            OpenLog.print(humid); OpenLog.print(";");
          #endif
          
          #ifdef USE_CO
            if(co_voltage > 0) OpenLog.print(co_voltage); OpenLog.print(";");
          #endif
          
          OpenLog.print("\n");          
          delay(15);
        #endif
      }
    #else
      #ifdef USE_OPENLOG
        OpenLog.print("<< ");
        OpenLog.print(dustDensity);
        OpenLog.print(";");
        #ifdef USE_BARO
           OpenLog.print(BMP.readTemperature());
           OpenLog.print(";");
           OpenLog.print(BMP.readPressure());
           OpenLog.print(";");
           delay(15);
        #endif
        #ifdef USE_HT
           OpenLog.print(temp_c);
           OpenLog.print(";");
           OpenLog.print(humid);
           OpenLog.print(";");
        #endif
      
        #ifdef USE_CO
          if(co_voltage > 0) OpenLog.print(co_voltage); OpenLog.print(";");
        #endif
        
        OpenLog.print("\n");
        delay(15);
      #endif
    #endif
    
    #ifdef DEBUG_ON
      #ifdef USE_BARO
        Serial.print("\nTemperature: "); Serial.print(BMP.readTemperature()); Serial.println(" [*C]");
        Serial.print("Pressure: "); Serial.print(BMP.readPressure()); Serial.println(" [Pa]");
        Serial.print("Altitude: "); Serial.print(BMP.readAltitude()); Serial.println(" [meters]");
      #endif
      Serial.print("Dust Density: ");  Serial.print(dustDensity); Serial.println(" [ug/m3]");
      Serial.print("Voltage Calculated: ");  Serial.print(calcVoltage); Serial.println(" [V]");
      Serial.print("Analogue Read: ");  Serial.print(voMeasured); Serial.println(" [0-1023]");
      #ifdef USE_HT
        Serial.print("\nTemperature: "); Serial.print(temp_c); Serial.println(" [*C]");
        Serial.print("Humidity: "); Serial.print(humid); Serial.println(" [%] RH");
      #endif
      
      #ifdef USE_CO
        Serial.print("\n");
        Serial.print("CO Sensor State: "); Serial.println(MQ7.Get_state());
        Serial.print("CO Voltage Reading: "); Serial.print(MQ7.Get_Voltage_reading());
        Serial.print("CO Current Reading: "); Serial.println(MQ7.Get_current_CO_reading());
      #endif
    #endif
 // }
  
  #ifdef USE_GPS
    GPSSerial.listen();
    //delay(1000);
  #endif

  #ifdef USE_GPS
    unsigned long start = millis(); // begin listening to the GPS
    // Listen to GPS for two seconds or until fix has been reached, whichever comes first
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
        Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
        Serial.println("New GPS Sentence received");
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



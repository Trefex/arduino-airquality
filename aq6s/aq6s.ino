/*
 Module using an Arduino Fio, a Sharp Optical Dust Sensor GP2Y1010AU0F,
 an Adafruit Ultimate GPS Breakout 3v3 and an OpenLog data logger.
 
 Blog: TO COME
 Code: https://github.com/Trefex/arduino-airquality/

 For Pin connections, please check the Blog or the github project page
 Authors: Cyrille Médard de Chardon (serialC), Christophe Trefois (Trefex)
 Changelog:
   2014-Feb-09: Fixed OpenLog problems due to dust sensor
   2013-Dec-11: Added CO code through library addition
   2013-Dec-22: Added CO Sensor
   2013-Dec-16: First version working.


 This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License. 
 To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ or send a letter 
 to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA. 
 */
 
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

// Allow for basic C++ functions
/*#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <utility.h>
#include <vector>

using namespace std;*/

#define USE_OPENLOG // disable to remove logging
#define USE_GPS // disable to remove GPS
#define USE_BARO // disable to remove Barometric sensor
#define DEBUG_ON // enable to output debugging information on normal Serial
#define GPSECHO false // put to true if you want to see raw GPS data flowing
#define USE_HT // comment to remove Humidity and temperature sensor
#define USE_CO // disable to remove CO sensor
#define USE_DUST // disable to remove DUST sensor

// Check which sensors are being used based on defines above
#ifdef USE_BARO
  //#include "Adafruit_BMP085.h"
  #include "Adafruit_BMP085_U.h"
  #include "Adafruit_Sensor.h"
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
#define HT_DATA_PIN 11 //D11 - Humidity/Temp sensor data
#define HT_SCK_PIN 13  //D13 - Humidity/Temp sensor serial clock

#define DUST_RX_PIN A6  //A6 - Dust sensor (Analog 6)
#define DUST_LED_PIN 12 //D12 - Dust sensor led

#define OPENLOG_RST_PIN 6 //D6 - Openlogger
#define OPENLOG_TX_PIN 7  //D7 - Openlogger
#define OPENLOG_RX_PIN 8  //D8 - Openlogger

#define ADAGPS_RX_PIN 2 // D2 - GPS RX to Pin ADAGPS_RX_PIN
#define ADAGPS_TX_PIN 3 // D3 - GPS TX to ADAGPS_TX_PIN
#define ADAGPS_RESET 10 // D10 - (Not used) Pull Low to switch off the module

#define ACTIVE_MONOX_LED_PIN 4 // D4 - CO LED Pin
#define ACTIVE_MONOX_PIN 5 // D5 - CO Switch Pin
#define READ_MONOX_PIN A7 // CO Read Pin
#define READ_COPSV_PIN A1 // CO Power Supply Voltage


#ifdef USE_CO
  COS_MQ7 MQ7(ACTIVE_MONOX_LED_PIN, ACTIVE_MONOX_PIN, READ_MONOX_PIN, READ_COPSV_PIN);
#endif

// Constructor declarations
#ifdef USE_HT
  SHT1x th_sensor(HT_DATA_PIN, HT_SCK_PIN);
#endif

#ifdef USE_OPENLOG
  SoftwareSerial OpenLog(OPENLOG_RX_PIN, OPENLOG_TX_PIN);  // RX, TX
#endif

#ifdef USE_BARO
  Adafruit_BMP085_Unified BMP = Adafruit_BMP085_Unified(10050);
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

float bmp_temp = 0;
float bmp_pres = 0;

uint32_t timer = millis();

void setup(){
  #ifdef DEBUG_ON
    // 115200 baud rate for connection
    Serial.begin(115200);
  #endif

  // if the initialization of the OpenLog is after the DustSensor output is set to high this doesn't work???!!
  #ifdef USE_OPENLOG
    // Setup and reset OpenLog
    pinMode(OPENLOG_RST_PIN, OUTPUT);
    OpenLog.begin(9600);

    // Reset OpenLog
    digitalWrite(OPENLOG_RST_PIN, LOW);
    delay(100);
    digitalWrite(OPENLOG_RST_PIN, HIGH);
    
    //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
    
    while(1) {
      if(OpenLog.available()) {

        // print how much is available
        Serial.print("There are ");Serial.print(OpenLog.available()); Serial.println(" characters available:");
        
        char olr = 'x';
        while( OpenLog.available() > 0) {
          olr = OpenLog.read();
          Serial.print(olr);
          
          if( olr == '<' ) openlog_ready = true;
        }
        Serial.print("\n");
        if( openlog_ready ) break;
        
        delay(2000);
      }
    }
    Serial.println("OpenLog is alive and recording.");
  #endif

  // define dust pins as output mode  
  #ifdef USE_DUST
    pinMode(DUST_LED_PIN, OUTPUT);
  #endif
  
  // define dust pins as output mode, reset GPS and begin GPS device
  #ifdef USE_GPS
    //pinMode(ADAGPS_RESET, OUTPUT);
  
    // reset the GPS (is not connected)
    //digitalWrite(ADAGPS_RESET, LOW);
    //delay(2000);
    //digitalWrite(ADAGPS_RESET, HIGH);
  
    GPS.begin(9600);
    delay(40);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
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
 
    #ifdef USE_DUST
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
    #endif

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
    
    #ifdef USE_BARO
      BMP.getTemperature(&bmp_temp);
      BMP.getPressure(&bmp_pres);
      bmp_pres = bmp_pres / 100.0;
    #endif
    
    #ifdef USE_OPENLOG
      OpenLog.print("<< ");
      #ifdef USE_GPS // If using GPS log data if a fix, otherwise write blanks
        if(GPS.fix) {
          // print date time
          OpenLog.print(GPS.day, DEC); OpenLog.print("/");
          OpenLog.print(GPS.month, DEC); OpenLog.print("/20");
          OpenLog.print(GPS.year, DEC); OpenLog.print(";");
          OpenLog.print(GPS.hour, DEC); OpenLog.print(":");
          delay(15);
          OpenLog.print(GPS.minute, DEC); OpenLog.print(":");
          OpenLog.print(GPS.seconds, DEC); OpenLog.print(";");
          // print location
          OpenLog.print(convertDegMinToDecDeg(GPS.latitude)); OpenLog.print(";"); OpenLog.print(GPS.lat); OpenLog.print(";"); 
          OpenLog.print(convertDegMinToDecDeg(GPS.longitude)); OpenLog.print(";"); OpenLog.print(GPS.lon); OpenLog.print(";");
        } else {
          OpenLog.print(";;;;;;");
        }
      #else
    
      #ifdef_USE_DUST
        // Dust Values
        OpenLog.print(dustDensity); OpenLog.print(";");
        delay(15);
      #endif
      
      #ifdef USE_BARO
         OpenLog.print(bmp_temp); OpenLog.print(";");
         OpenLog.print(bmp_pres); OpenLog.print(";");
         delay(15);
      #endif
      
      #ifdef USE_HT
        OpenLog.print(temp_c); OpenLog.print(";");
        OpenLog.print(humid); OpenLog.print(";");
      #endif
      
      #ifdef USE_CO
        OpenLog.print(co_voltage); OpenLog.print(";");
      #endif
      
      OpenLog.print("\n");          
      delay(15);
    #endif

    // Debug/test printout of sensor values    
    #ifdef DEBUG_ON
      #ifdef USE_BARO
        Serial.print("\nTemperature: "); Serial.print(bmp_temp); Serial.println(" [*C]");
        Serial.print("Pressure: "); Serial.print(bmp_pres); Serial.println(" [hPa]");
      #endif
      
      #ifdef USE_DUST
        Serial.print("Dust Density: ");  Serial.print(dustDensity); Serial.println(" [ug/m3]");
        Serial.print("Voltage Calculated: ");  Serial.print(calcVoltage); Serial.println(" [V]");
        Serial.print("Analogue Read: ");  Serial.print(voMeasured); Serial.println(" [0-1023]");
      #endif
      
      #ifdef USE_HT
        Serial.print("\nTemperature: "); Serial.print(temp_c); Serial.println(" [*C]");
        Serial.print("Humidity: "); Serial.print(humid); Serial.println(" [%] RH");
      #endif
      
      #ifdef USE_CO
        Serial.print("\n");
        Serial.print("CO Sensor State: "); Serial.println(MQ7.Get_state());
        Serial.print("CO Voltage Reading: "); Serial.println(MQ7.Get_Voltage_reading());
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
      //Serial.println("Before newNMEA");
      if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences! 
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
        //Serial.println("New GPS Sentence received");
        //gpsReady = true;
        //break;
        //Serial.println("Before GPS.lastNMEA");
        if (GPS.parse(GPS.lastNMEA())){   // this also sets the newNMEAreceived() flag to false
        //Serial.println("Before GPS fix");
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
                Serial.print(convertDegMinToDecDeg(GPS.latitude),6); Serial.print(GPS.lat);
                Serial.print(", "); 
                Serial.print(convertDegMinToDecDeg(GPS.longitude),6); Serial.println(GPS.lon);
                
                Serial.print("Speed (knots): "); Serial.println(GPS.speed);
                Serial.print("Angle: "); Serial.println(GPS.angle);
                Serial.print("Altitude: "); Serial.println(GPS.altitude);
                Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
                break;
            #endif
          } else {
            #ifdef DEBUG_ON
              Serial.print("No GPS fix achieved.");
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

//void clearSerial() {
//  #ifdef DEBUG_ON
//    while (Serial.read() >= 0) {;}
//  #endif
//  #ifdef USE_GPS
//    while(GPSSerial.read() >= 0) {;}
//  #endif
//  //while(OpenLog.read() >= 0){;}
//}

// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
double convertDegMinToDecDeg (float degMin) {
  double minutes = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  minutes = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( minutes / 60 );
 
  return decDeg;
}



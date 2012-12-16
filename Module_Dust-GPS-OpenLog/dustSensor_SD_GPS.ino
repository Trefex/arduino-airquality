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
#include <Adafruit_GPS.h>
 
#define measurePin 6
#define ledPower 12

#define USE_OPENLOG // disable to remove logging
#define DEBUG_ON // enable to output debugging information on normal Serial
#define GPSECHO false // put to true if you want to see raw GPS data flowing

#define OPENLOG_RST_PIN 6
#define OPENLOG_TX_PIN 7
#define OPENLOG_RX_PIN 8
#define ADAGPS_RX_PIN 2 // GPS RX to Pin ADAGPS_RX_PIN
#define ADAGPS_TX_PIN 3 // GPS TX to ADAGPS_TX_PIN
#define ADAGPS_RESET 10 // Pull Low to switch off the module

#ifdef USE_OPENLOG
  SoftwareSerial OpenLog(OPENLOG_RX_PIN, OPENLOG_TX_PIN);
#endif

// Create Soft Serial GPS handle
SoftwareSerial GPSSerial(ADAGPS_TX_PIN, ADAGPS_RX_PIN);

// Create GPS handle using GPS Soft Serial
Adafruit_GPS GPS(&GPSSerial);

#ifdef USE_OPENLOG
  //static const int resetOpenLog = OPENLOG_RST_PIN;
  boolean openlog_ready = false;
#endif

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
boolean gpsReady = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

void setup(){
  Serial.begin(115200);
  pinMode(ledPower,OUTPUT);
  pinMode(ADAGPS_RESET, OUTPUT);
  
  digitalWrite(ADAGPS_RESET, LOW);
  delay(2000);
  digitalWrite(ADAGPS_RESET, HIGH);
  
  GPS.begin(9600);
  delay(40);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  
  #ifdef USE_OPENLOG
    // Setup OpenLog
    OpenLog.begin(9600);
    pinMode(OPENLOG_RST_PIN, OUTPUT);  
  #endif

  // do not use intterupts  
  useInterrupt(false);

  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop(){
  Serial.println("Being of loop");
  if (millis() - timer > 2000) { 
    
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
    if(GPS.fix) {
      #ifdef USE_OPENLOG
        OpenLog.print("<< ");
        OpenLog.print(dustDensity);
        OpenLog.print(";");
        OpenLog.print(GPS.latitude, 4); OpenLog.print(GPS.lat);
        OpenLog.print(";"); 
        OpenLog.print(GPS.longitude, 4); OpenLog.println(GPS.lon);
        delay(15);
      #endif
    }
    
    #ifdef DEBUG_ON 
      Serial.print("\nRaw Signal Value (0-1023): ");  Serial.print(voMeasured);
      Serial.print(" - Voltage: ");  Serial.print(calcVoltage);
      Serial.print(" - Dust Density [ug/m3]: ");  Serial.println(dustDensity);
    #endif
  }
  
  GPSSerial.listen();
  //delay(1000);

  unsigned long start = millis(); // begin listening to the GPS
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
     // return;  // we can fail to parse a sentence in which case we should just wait for another
      
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
        #endif
      }
    }
    }
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if ((millis() - timer > 2000) && gpsReady) { 

    
    Serial.print("\nRaw Signal Value (0-1023): ");
    Serial.print(voMeasured);
    
    Serial.print(" - Voltage: ");
    Serial.print(calcVoltage);
    
    Serial.print(" - Dust Density [ug/m3]: ");
    Serial.println(dustDensity);
    
    timer = millis(); // reset the timer
    
    #ifdef USE_OPENLOG
      OpenLog.print(";");
      OpenLog.print(GPS.hour, DEC); OpenLog.print(':');
      OpenLog.print(GPS.minute, DEC); OpenLog.print(':');
      OpenLog.print(GPS.seconds, DEC); OpenLog.print('.');
      OpenLog.print(GPS.milliseconds);
      OpenLog.print(";");
      OpenLog.print(GPS.day, DEC); OpenLog.print('/');
      OpenLog.print(GPS.month, DEC); OpenLog.print("/20");
      OpenLog.println(GPS.year, DEC);
      delay(15);
    #endif
    
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
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
    gpsReady = false;
  }
  //clearSerial();
  //delay(400);
}

void clearSerial() {
  while (Serial.read() >= 0) {;}
  while(GPSSerial.read() >= 0) {;}
  //while(OpenLog.read() >= 0){;}
}



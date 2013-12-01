/*
 Module using an Arduino Fio and a MQ-7 on Parallax Gas Sensor Module.
 A seperate power source is used as the Fio cannot source enough current 
 for the MQ-7 module.
 
 Blog: http://arduinodev.woofex.net/
 Code: https://github.com/Trefex/arduino-airquality/

 For Pin connections, please check the Blog or the github project page
 Authors: Cyrille Médard de Chardon (serialC), Christophe Trefois (Trefex)
 Changelog:
   2013-Dec-01: First version with working codebase

 This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License. 
 To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ or send a letter 
 to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA. 
 */

// Pin definitions
#define ACTIVE_MONOX_PIN 11
#define READ_MONOX_PIN  7

// literal/constants definitions
#define HIGH_V_TIME 60 //60
#define LOW_V_TIME 90 //90

// Variable initializations
int reading; // holds value [0 - 1023]

void setup() {                
  // initialize pins' modes
  pinMode(ACTIVE_MONOX_PIN, OUTPUT);     
  pinMode(READ_MONOX_PIN, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  
  int x;
  
  // turn on heating coil to 'clean' it/remove humidity, wait
  pinMode(ACTIVE_MONOX_PIN, OUTPUT);
  analogWrite(ACTIVE_MONOX_PIN, 128);
  Serial.println("\nHot/purge readings:");
  for( x = 0; x < HIGH_V_TIME; x++) {
    delay(1000);
    Serial.println(analogRead(READ_MONOX_PIN));
  }
  
  // turn off sensor and let it cool
  analogWrite(ACTIVE_MONOX_PIN, 0);
  pinMode(ACTIVE_MONOX_PIN, INPUT);
  Serial.println("Cool/sense readings:");
  for( x = 0; x < LOW_V_TIME; x++) {
    delay(1000);
    Serial.println(analogRead(READ_MONOX_PIN));
  }
  
  // now measure, before we turn it the heating coil on again
  reading = analogRead(READ_MONOX_PIN);
  Serial.print("\nRecorded reading: ");
  Serial.println(reading);
}




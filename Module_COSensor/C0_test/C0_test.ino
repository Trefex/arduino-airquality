/*
 Module using an Arduino Fio and a MQ-7 on Parallax Gas Sensor Module.
 A seperate power source is used as the Fio cannot source enough current 
 for the MQ-7 module.
 
 Blog: http://arduinodev.woofex.net/
 Code: https://github.com/Trefex/arduino-airquality/

 For Pin connections, please check the Blog or the github project page
 Authors: Cyrille Médard de Chardon (serialC), Christophe Trefois (Trefex)
 Changelog:
   2013-Dec-22: First correctly working version with codebase

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
  analogWrite(ACTIVE_MONOX_PIN, 255);
  
  for( x = 0; x < HIGH_V_TIME; x++) {
    Serial.print("H,");
    Serial.println(analogRead(READ_MONOX_PIN));
    delay(1000);
  }
  
  // Turn down heater from 5V to 1.4V for 'cooling'
  // We need to allow 1.4 V through the heating coil, down from 5V
  // We can consider putting 1.4V as putting 5V only 1.4/5 of the time
  // Because we are using PWM to open a transistor, we can allow current to flow from collector
  // to emitter for 1.4/5 of the time, this means a PWM value (which ranges from 0-255) of 1.4/5*255 = 71.4
  // We are in fact allowing the 5V go go through 71 times for 1/255 of a second - this creates an avg of 1.4V
  pinMode(ACTIVE_MONOX_PIN, INPUT);
  analogWrite(ACTIVE_MONOX_PIN, 71);
  
  for( x = 0; x < LOW_V_TIME; x++) {
    Serial.print("C,");
    Serial.println(analogRead(READ_MONOX_PIN));
    delay(1000);
  }
  
  // now measure
  // turn thw PWM pin to 0 so that we don't measure while in the middle of a heating pulse
  analogWrite(ACTIVE_MONOX_PIN, 0);
  
  reading = analogRead(READ_MONOX_PIN);
  Serial.print("R,");
  Serial.println(reading);
  
  // restart loop and turn the heating coil back on again
}




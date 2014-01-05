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
   2014-Jan-04: Updated existing library to facilitate usage

 This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License. 
 To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ or send a letter 
 to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA. 
 */

#include "COS_MQ7.h"

// Pin definitions
#define ACTIVE_MONOX_LED_PIN 13
#define ACTIVE_MONOX_PIN 11
#define READ_MONOX_PIN A7
#define READ_COPSV_PIN A1

// Variable initializations
int reading, voltage;

// create CO sensor object
COS_MQ7 MQ7(ACTIVE_MONOX_LED_PIN, ACTIVE_MONOX_PIN, READ_MONOX_PIN, READ_COPSV_PIN);

void setup() {                
  Serial.begin(9600);
}

void loop() {
  MQ7.Power_cycle();
  
  // for testing
  Serial.print(MQ7.Get_state());
  Serial.print(',');
  Serial.print(MQ7.Get_Voltage_reading());
  Serial.print(',');
  Serial.println(MQ7.Get_current_CO_reading());
  
  // Only record 'good' reading
  if(MQ7.Get_state() == 4) {
    voltage = MQ7.Get_CO_reading();
  }
  
  // Do something when CO psu voltage is too low
  if(MQ7.Get_Voltage_reading() < 400) {
    Serial.println("CO sensor power supply voltage is too low.");
    // do something
  }
  
  delay(1000);
}

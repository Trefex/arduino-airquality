/*
  #############################################
  C0 Sensor (MQ-7 on Parallax Gas Sensor Module
  basic test using Arduino Fio (3.3V) with additional
  battery source
  #############################################
  
  When heating coil, the arduino fio LED is off, and on during coil cooling.
  See output for readings showing if heating is on and read value
 */

// Pin definitions
#define ACTIVE_MONOX_PIN 13
#define READ_MONOX_PIN  7

// literal/constants definitions
#define HIGH_V_TIME 60
#define LOW_V_TIME 90

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
  
  // NOTE: HSW input is active-low
  
  // turn off sensor and let it cool
  digitalWrite(ACTIVE_MONOX_PIN, HIGH);
  Serial.println("Cool readings:");
  for( x = 0; x < LOW_V_TIME; x++) {
    delay(1000);
    Serial.println(analogRead(READ_MONOX_PIN));
  }
  
  // turn on sensor to 'clean' it/remove humidity, wait
  digitalWrite(ACTIVE_MONOX_PIN, LOW);
  Serial.println("\nHot readings:");
  for( x = 0; x < HIGH_V_TIME; x++) {
    delay(1000);
    Serial.println(analogRead(READ_MONOX_PIN));
  }
  
  // now measure, before we turn it off again
  reading = analogRead(READ_MONOX_PIN);
  Serial.print("\nRecorded reading: ");
  Serial.println(reading);
}







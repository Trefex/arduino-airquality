/*
  #############################################
  C0 Sensor (MQ-7 on Parallax Gas Sensor Module
  basic test using Arduino Fio (3.3V) with additional
  battery/power source
  #############################################
  
  When heating the coil
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




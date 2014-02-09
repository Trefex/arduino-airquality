/* OpenLog Test using SoftwareSerial.h 
   See OpenLog_ReadExample.ino for more examples of interactions using SoftwareSerial
   https://github.com/jamescw/OpenGPSLog/blob/master/OpenLog_ReadExample/OpenLog_ReadExample.ino
*/
 
#include <SoftwareSerial.h>
 
#define OPENLOG_RST_PIN 6 // D6 - Openlogger
#define OPENLOG_TX_PIN 7  // D7 - Openlogger
#define OPENLOG_RX_PIN 8  // D8 - Openlogger

SoftwareSerial OpenLog(OPENLOG_RX_PIN, OPENLOG_TX_PIN); // RX, TX
boolean openlog_ready = false;

void setup() {
  Serial.begin(115200);
  
  pinMode(OPENLOG_RST_PIN, OUTPUT);
  OpenLog.begin(9600);
  
  //Reset OpenLog
  digitalWrite(OPENLOG_RST_PIN, LOW);
  delay(100);
  digitalWrite(OPENLOG_RST_PIN, HIGH);

  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
    while(1) {
      if(OpenLog.available()) {

        // print how much is available
        Serial.print("There are ");Serial.print(OpenLog.available()); Serial.println(" characters available.");
        
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
}

void loop() {
  OpenLog.println("Hello");
  Serial.println("Wrote: 'Hello' on sd card.");
  delay(1000);
}

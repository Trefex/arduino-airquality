/**
 * Humidity - Temperature sensor module
 * Cyrille Medard de Chardon, Christophe Trefois
 */

#include "SHT1x.h"
#define dataPin 7
#define sckPin 8 //serial clock

// We are using the SHT10
// Humid accuracy +/- 5%
// Steady accuracy between 10-80
// example at 10/90 +/- 6%, 0/100 +/- 7.5%

// Temp accuracy +/- .5 degrees celsius
// Temp error increases more as we get farther from 25 cels.
// example: @ 0/50 degrees, +/- 1.2 degrees

SHT1x th_sensor(dataPin, sckPin);

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting up");
}

void loop()
{  
  float temp_c;
  float humid;
  
  // Read values from the sensor
  humid = th_sensor.readHumidity();
  // Since the humidity reading requires the temperature we simply
  // retrieve the reading capture from the readHumidity() call. See the lib.
  temp_c = th_sensor.retrieveTemperatureC();
  
  // Print data
  Serial.print("Temperature: ");
  Serial.print(temp_c);
  Serial.print(", Humidity: ");
  Serial.println(humid);

  delay(4000);
}

/**
 * SHT1x Library
 *
 * Modified by Cyrille Medard de Chardon, Christophe Terfois for 3V3 Arduino Fio
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 * Based on previous work by:
 *    Maurice Ribble: <www.glacialwanderer.com/hobbyrobotics/?p=5>
 *    Wayne ?: <ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html>
 *
 * Manages communication with SHT1x series (SHT10, SHT11, SHT15)
 * temperature / humidity sensors from Sensirion (www.sensirion.com).
 */
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "SHT1x.h"

SHT1x::SHT1x(int dataPin, int clockPin)
{
  _dataPin = dataPin;
  _clockPin = clockPin;
  _temperatureC = -273.0;
}

/* ================  Public methods ================ */

/**
 * To be called after readHumidity() which calls readTemperatureC()
 */

float SHT1x::retrieveTemperatureC()
{
  return _temperatureC;
}

/**
 * Reads the current temperature in degrees Celsius
 */
float SHT1x::readTemperatureC()
{
  int _val;                // Raw value returned from sensor
  float _temperature;      // Temperature derived from raw value

  // Conversion coefficients from SHT1x V5 datasheet
  const float D1 = -39.66; // for 14 bit @ 3V3
  const float D2 =   0.01; // for 14 Bit DEGC

  // Fetch raw value
  _val = readTemperatureRaw();

  // Convert raw value to degrees Celsius
  _temperature = (_val * D2) + D1;

  // Set global variable
  _temperatureC = _temperature;

  //return (_val);
  return (_temperature);
}

/**
 * Reads current temperature-corrected relative humidity
 */
float SHT1x::readHumidity()
{
  int _val;                    // Raw humidity value returned from sensor
  float _linearHumidity;       // Humidity with linear correction applied
  float _correctedHumidity;    // Temperature-corrected humidity
  float _temperature;          // Raw temperature value

  // Conversion coefficients from SHT1x V5 datasheet
//  const float C1 = -4.0;       // for 12 Bit
//  const float C2 =  0.0405;    // for 12 Bit
//  const float C3 = -0.0000028; // for 12 Bit
  // Fio coefficients for SHT1x with 3.3V
  const float C1 = -2.0468;	// for 12 bit
  const float C2 = 0.0367;	// for 12 bit
  const float C3 = -0.0000015955;// for 12 bit, see Table 6 in SHT1x V5 datasheet
  const float T1 =  0.01;      // for 14 Bit - independent of voltage
  const float T2 =  0.00008;   // for 14 Bit - independent of voltage

  // Command to send to the SHT1x to request humidity
  int _gHumidCmd = 0b00000101;

  // Fetch the value from the sensor
  sendCommandSHT(_gHumidCmd, _dataPin, _clockPin);
  waitForResultSHT(_dataPin);
  _val = getData16SHT(_dataPin, _clockPin);

  // Apply linear conversion to raw value
  _linearHumidity = C1 + C2 * _val + C3 * _val * _val;

  // Get current temperature for humidity correction
  _temperature = readTemperatureC();

  // Correct humidity value for current temperature
  _correctedHumidity = (_temperature - 25.0 ) * (T1 + T2 * _val) + _linearHumidity;

  return (_correctedHumidity);
}


/* ================  Private methods ================ */

/**
 * Reads the current raw temperature value
 */
float SHT1x::readTemperatureRaw()
{
  int _val;

  // Command to send to the SHT1x to request Temperature
  int _gTempCmd  = 0b00000011;

  sendCommandSHT(_gTempCmd, _dataPin, _clockPin);
  waitForResultSHT(_dataPin);
  _val = getData16SHT(_dataPin, _clockPin);
  //skipCrcSHT(_dataPin, _clockPin);

  return (_val);
}

/**
 */
int SHT1x::shiftIn(int _dataPin, int _clockPin, int _numBits)
{
  int ret = 0;
  int i;

  for (i=0; i<_numBits; ++i)
  {
     digitalWrite(_clockPin, HIGH);
     delay(10);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
     ret = ret*2 + digitalRead(_dataPin);
     digitalWrite(_clockPin, LOW);
  }

  return(ret);
}

/**
 */
void SHT1x::sendCommandSHT(int _command, int _dataPin, int _clockPin)
{
  int ack;

  // Transmission Start
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, LOW);

  // The command (3 msb are addresses and must be 000, and last 5 bits are command)
  // MSB = Most Significant Bit, Sends _command one bit at a time from left to right
  shiftOut(_dataPin, _clockPin, MSBFIRST, _command);

  // Verify we get the correct ack
  digitalWrite(_clockPin, HIGH);
  pinMode(_dataPin, INPUT);
  ack = digitalRead(_dataPin);
  if (ack != LOW) {
    Serial.println("Ack Error 0");
  }
  digitalWrite(_clockPin, LOW);
  ack = digitalRead(_dataPin);
  if (ack != HIGH) {
    Serial.println("Ack Error 1");
  }
}

/**
 */
void SHT1x::waitForResultSHT(int _dataPin)
{
  int i;
  int ack;

  pinMode(_dataPin, INPUT);

  for(i= 0; i < 100; ++i)
  {
    delay(10);
    ack = digitalRead(_dataPin);

    if (ack == LOW) {
      break;
    }
  }

  if (ack == HIGH) {
    Serial.println("Ack Error 2");
  }
}

/**
 */
int SHT1x::getData16SHT(int _dataPin, int _clockPin)
{
  int val;

  // Get the 8 left/most significant bits
  pinMode(_dataPin, INPUT);
  pinMode(_clockPin, OUTPUT);
  val = shiftIn(_dataPin, _clockPin, 8);
  // ~25 degrees celsius == 25
  val <<= 8;  //shift 8 bits left
  // ~25 degrees celsius == 6400

  // Send the required DATA ACK, set to LOW
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, LOW);

  // Send SCK ACK
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);

  // Get the least significant bits
  pinMode(_dataPin, INPUT);
  // bit-wise OR, like adding
  val |= shiftIn(_dataPin, _clockPin, 8);

  // I don't understand why this is helping/necessary
  // tempermental! It's fine without now?
  // Cable length?
  //delay(1);

  // We are not performing CRC so don't drop DATA to LOW
  // ACK receipt of LSb
  // pinMode of SCK already set to OUTPUT
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);

  return val;
}

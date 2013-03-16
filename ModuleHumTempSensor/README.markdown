SHT1x Temperature / Humidity Sensor Library for Arduino
=======================================================
Heavily modified by Cyrille Medard de Chardon from the original creator's version:

Copyright 2009 Jonathan Oxer jon@oxer.com.au / http://www.practicalarduino.com  
Copyright 2008 Maurice Ribble ribblem@yahoo.com / http://www.glacialwanderer.com

It now provides a simple interface to the SHT10 sensor made to function with the Arduino Fio (3.3V)

Installation
------------
Put this directory in your sketchbook and open the ino file.

Usage
-----
The library is instantiated as an object with methods provided to read
relative humidity and temperature. Include it in your sketch and then
create an object, specifying the pins to use for communication with the
sensor:

    #include "SHT1x.h"
    #define dataPin 10
    #define clockPin 11
    SHT1x sht10(dataPin, clockPin);

You can then call methods on that object within your program. In this
example we created an object called "sht10", but it could have been
called whatever you like.

### readTemperatureC() ###

Returns a float within the valid range of the sensor of -40 to +123.8C.
A value of -273 is returned in the event of a communication error with
the sensor.

Example:

    float tempC = sht10.readTemperatureC();

### retrieveTemperatureC() ###

Returns a float of the temperature in celcius that was measured during the humidity measurement. This prevents needing two calls to the temperature sensor. Call the humidyt function readHumidity() then retrieveTemperatureC().

### readHumidity() ###

Returns a float within the valid range of the sensor of 0 to 100%.
A negative value is returned in the event of a communication error with
the sensor.

Example:

    float humidity = sht1x.readHumidity();

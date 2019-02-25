arduino-airquality
==================
 
In this project, we are building a mobile Arduino Fio based platform to measure air quality.

We currently have multiple components working correctly with the Arduino Fio:

- [x] GPS sensor - Adafruit Ultimate GPS Breakout v3 and the OpenLog
- [x] Dust sensor - Sharp Optical Dust Sensor
- [x] Humidity and temperature sensor - SHT10 by sensiron/DFRobot
- [x] Carbon monoxide sensor - Parallax CO sensor
- [x] Barometric pressure - BMP085 Barometric Pressure/Temperature/Altitude Sensor

We are working on combining them all into a nice light package.

Still to do:
- [x] Create circuit board sketch and etch it
- [x] Solder everything on board
- [x] Build enclosure

Next version improvements:

1. Use smaller arduino board (Nano?) potentially one with GPS builtin.

2. Decide if we want a 5V board.

3. Use 7.4V LI-ion flat and rechargeable battery pack.

4. Use only one voltage regulator if we stay with 3.3V.

5. Move voltage splitter to monitor voltage to between battery and voltage regulator.

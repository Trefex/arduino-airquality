arduino-airquality
==================

In this project, we are building an Arduino Fio based platform to measure air quality (carbon monoxide and dust levels) and also tracking location using a GPS sensor.

We currently have multiple components working correctly with the Arduino Fio:

- [x] GPS sensor - Adafruit Ultimate GPS Breakout v3 and the OpenLog
- [x] Dust sensor - Sharp Optical Dust Sensor
- [x] Humidity and temperature sensor - SHT10 by sensiron/DFRobot
- [ ] Carbon monoxide sensor - Parallax CO sensor (in progress)

We are working on combining them all into a nice light package.

To do:
- Determine if Fio has a voltage limit, our dust sensor could output a reading of up to ~3.75V
	It seems the only limit is on current, eg 20 mA per Pin, and 150 mA total for the board.
- Cook the CO sensor for 24-48 hours
- Add the CO sensor to bread board
- Add additional power supply for C0
- Figure out how to manage the higher voltage requirements of the C0 sensor
- Add CO code to main code base
- Add GPS conversion code to main code base

Long term steps
- Solder everything on prototype board
- Build enclosure
- Evaluate the need of adding additional sensors, such as MQ-131 for Ozone 

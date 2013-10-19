
/*  
	CS_MQ7_02.h - Library for reading the MQ-7 Carbon Monoxide Sensor
	Breakout, as part of the Citizen Sensor project.	
	http://citizensensor.cc
	
	Released into the public domain.
	
	Created by J Saavedra, October 2010.
	http://jos.ph

*/

#ifndef CS_MQ7_h
#define CS_MQ7_h

#include "WProgram.h"

class CS_MQ7{

	public:
		
		CS_MQ7(int CoTogPin, int CoIndicatorPin);
		void CoPwrCycler();
		boolean CurrentState();
		
		unsigned long time;
		unsigned long currTime;
		unsigned long prevTime;
		unsigned long currCoPwrTimer;

		boolean CoPwrState;
		
	private:
		int _CoIndicatorPin;
		int _CoTogPin;
	
	};

#endif
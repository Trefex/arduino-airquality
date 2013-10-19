
/*  
	CS_MQ7_02.h - Library for reading the MQ-7 Carbon Monoxide Sensor
	Breakout, as part of the Citizen Sensor project.	
	http://citizensensor.cc
	
	Released into the public domain.
	
	Created by J Saavedra, October 2010.
	http://jos.ph

*/

#include "WProgram.h"
#include "CS_MQ7.h"

CS_MQ7::CS_MQ7(int CoTogPin, int CoIndicatorPin){

	pinMode(CoIndicatorPin, OUTPUT);
	pinMode(CoTogPin, OUTPUT);
	
	_CoIndicatorPin = CoIndicatorPin;
	_CoTogPin = CoTogPin;
	
	time = 0;
	currTime = 0;
	prevTime = 0;
	currCoPwrTimer = 0;
	CoPwrState = LOW;
  	currCoPwrTimer = 500;
	
	}

void CS_MQ7::CoPwrCycler(){
  
  currTime = millis();
   
  if (currTime - prevTime > currCoPwrTimer){
    prevTime = currTime;
    
    if(CoPwrState == LOW){
      CoPwrState = HIGH;
      currCoPwrTimer = 60000;  //60 seconds at 5v
    }
    else{
      CoPwrState = LOW;
      currCoPwrTimer = 90000;  //90 seconds at 1.4v
    }
    digitalWrite(_CoIndicatorPin, CoPwrState);
    digitalWrite(_CoTogPin, CoPwrState);
  }
}

boolean CS_MQ7::CurrentState(){
	
	if(CoPwrState == LOW){
		return false;
	}
	else{
		return true;
	}
}
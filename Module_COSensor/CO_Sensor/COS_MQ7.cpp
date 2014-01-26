/*  
	CO Sensor library
	COS_MQ7.h - Library for reading the MQ-7 Carbon Monoxide Sensor	
	Created by J Saavedra, October 2010.
	http://jos.ph
	Modified by Cyrille Medard de Chardon, Christophe Trefois 2014.
*/

#include "COS_MQ7.h"

// Constructor
COS_MQ7::COS_MQ7(int LED_Indicator_Pin, int PWM_Tog_Pin, int CO_Reading_Pin, int V_Reading_Pin){

	pinMode(LED_Indicator_Pin, OUTPUT);
	pinMode(PWM_Tog_Pin, OUTPUT);
	pinMode(CO_Reading_Pin, INPUT);
	pinMode(V_Reading_Pin, INPUT);
	
	_LED_Indicator_Pin = LED_Indicator_Pin;
	_PWM_Tog_Pin = PWM_Tog_Pin;
	_CO_Reading_Pin = CO_Reading_Pin;
	_V_Reading_Pin = V_Reading_Pin;
	
	State = 0;
	Current_time = millis();
	Last_time = Current_time;
  	State_duration = 5; // Wait 5 seconds before starting
	Reading = -1;
	Voltage = -1;
        current_reading = -1;
}

void COS_MQ7::Power_cycle(){
  
  Current_time = millis();

  // get current reading from CO and voltage
  current_reading = analogRead(_CO_Reading_Pin);
  Voltage = analogRead(_V_Reading_Pin);
  
  // see if enough time has passed to change state
  if ( (Current_time - Last_time) > (State_duration * 1000) ){
    Last_time = Current_time;
    State_duration = 0;

    // complete one of the following based on State
    switch(State) {

      // initialization purge - only runs on boot/launch
      case 0:
        State = 1;
        State_duration = 500;  // 500 seconds at 5v to do initial purge
        digitalWrite(_LED_Indicator_Pin, HIGH);
        digitalWrite(_PWM_Tog_Pin, HIGH);
        break;

      // start heating
      case 1:
        State = 2;
        State_duration = 60;  // 60 seconds at 5v
        digitalWrite(_LED_Indicator_Pin, HIGH);
        digitalWrite(_PWM_Tog_Pin, HIGH);
        break;
  
      // start cooling
      case 2:
        State = 3;
        State_duration = 90;  //90 seconds at 1.4v
        digitalWrite(_LED_Indicator_Pin, LOW);
        analogWrite(_PWM_Tog_Pin, 71); // need to use analogWrite to digital PWM pin
        break;
  
      // cooling is finished, read 'correct' reading
      case 3:
        State = 4;
        Reading = current_reading;
        break;
  
      case 4:
        // indefinite state until the value is read
        break;
    }
  }
}

int COS_MQ7::Get_CO_reading(){
	// will begin heating again after next Power_cycle() call
        int tmpRet;
	State = 1;
        tmpRet = Reading;
	Reading = -1;
	return tmpRet;
}

int COS_MQ7::Get_current_CO_reading(){
	return current_reading;
}

int COS_MQ7::Get_Voltage_reading(){
	return Voltage;
}

int COS_MQ7::Get_state(){
	return State;
}

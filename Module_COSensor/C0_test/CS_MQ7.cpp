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
	
	Current_time = 0;
	Last_time = 0;
	Power_state = LOW;
  	State_duration = 500;//seconds	
	Reading = -1;
	Voltage = -1;
}

void COS_MQ7::Power_cycle(){
  
  Current_time = millis();
  
  // see if enough time has passed to change state
  if ( (Current_time - Last_time) > (State_duration * 1000) ){
    Last_time = Current_time;

    // initialization heating
    if(State == 0){
      State = 1;
      State_duration = 500; // 500 seconds to clean coil
      digitalWrite(_LED_Indicator_Pin, HIGH);
      digitalWrite(_PWM_Tog_Pin, HIGH);
    }

    // start heating
    if(State == 1){
      State = 2;
      State_duration = 60;  // 60 seconds at 5v
      digitalWrite(_LED_Indicator_Pin, HIGH);
      digitalWrite(_PWM_Tog_Pin, HIGH);
    }

    // start cooling
    if(State == 2){
      State = 3;
      State_duration = 90;  //90 seconds at 1.4v
      digitalWrite(_LED_Indicator_Pin, HIGH);
      analogWrite(_PWM_Tog_Pin, 71); // need to use analogWrite to digital PWM pin
    }

    // cooling is finished, read
    if(State == 3){
      State = 4;
      Reading = analogRead(READ_MONOX_PIN);
    }

    if(State == 4){
      // indefinite state until the value is read
    }
  }
}

int COS_MQ7::Get_CO_reading(){
	// will begin heating again after next Power_cycle() call
	State = 1;
	return Reading;
}

int COS_MQ7::Get_Voltage_reading(){
	return Voltage;
}

int COS_MQ7::Get_state(){
	return State;
}

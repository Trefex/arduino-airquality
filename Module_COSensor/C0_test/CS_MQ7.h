/*  
	COS_MQ7.h - Library for reading the MQ-7 Carbon Monoxide Sensor
	Released into the public domain.
	Created by J Saavedra, October 2010.
	http://jos.ph
	Modified by Cyrille Medard de Chardon, Christophe Trefois 2014.
*/

#ifndef COS_MQ7_h
#define COS_MQ7_h

class COS_MQ7{

	public:
		COS_MQ7(int LED_Indicator_Pin, int PWM_Tog_Pin, int CO_Reading_Pin, int V_Reading_Pin);
		void Power_cycle();
		int Get_state();
		int Get_CO_reading();
		int Get_Voltage_reading();

	private:
		unsigned long Current_time;
		unsigned long Last_time;
		unsigned long State_duration;

		int _LED_Indicator_Pin;
		int _PWM_Tog_Pin;
		int _CO_Reading_Pin;
		int _V_Reading_Pin;

		int Reading;
		int Voltage;
	};

#endif

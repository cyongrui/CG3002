/*****************************************************
  MULTIMETER
*****************************************************/
#include <Arduino.h>
#include "power.h"

const int VOLTMETER_PIN=0;
const int AMMETER_PIN=1;

//const int VOLTMETER_PIN=A0;
//const int AMMETER_PIN=A1;

// INA169 constants and variables
const float RS = 0.1; // Shunt resistor value (in ohms)
// Voltmeter constants and variables
const float ratio = 2.0; // ratio = 1 / (R2 / (R1 + R2))

Power measurePower() {
  Power power;
	int inputVolt = analogRead(VOLTMETER_PIN);
	float voltOut = (inputVolt * VOLTAGE_REF) / 1024.0;
	float voltIn = voltOut * ratio; 

	int inputAmp = analogRead(AMMETER_PIN);
	float current = ((inputAmp * VOLTAGE_REF) / 1024.0) / (10 * RS);
	current = 0.35 + inputAmp * 0.003;
  
	//float power = voltIn * current;		
	power.voltage = voltIn;
  power.current = current;
	return power;
}




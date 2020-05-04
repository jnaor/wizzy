#include <Adafruit_NeoPixel.h>
#include "BugSignalingMessage.h"
#include "WizzyDefinitions.h"
 
#pragma once

class VibrationMotor {
  public:
    byte motorID;
	uint8_t motorPin;
	bool isActive = false;
    
	unsigned long currentRepetition = 0;
	unsigned long lastStateChange = 0;
	float jumpValue;
	float currentPower;
	int currentState=0;
	
	MotorParams localParams;
	
	VibrationMotor(uint8_t id, uint8_t pin);
    void turnOff();     
	void iterateSequence();
	void beginSequence();
	void setPower(uint8_t power);
};

VibrationMotor::VibrationMotor(uint8_t id, uint8_t pin) {
  motorID = id; 
  motorPin = pin;
  pinMode(pin, OUTPUT);
  setPower(0);
}

void VibrationMotor::turnOff() {
	setPower(0);
	currentPower = 0;
	currentState = 0;
}

void VibrationMotor::setPower(uint8_t power){
  analogWrite(motorPin, power);
  currentPower = power;
}

void VibrationMotor::iterateSequence()
{
	unsigned long now = millis();	
	unsigned long stateDiff = now - lastStateChange;
		
	// Serial.print(stateDiff);
	// Serial.print(' ');
	// Serial.print(currentState);
	// Serial.print(' ');
	// Serial.print(currentPower);
	// Serial.print(' ');
	// Serial.print(currentRepetition);
	// Serial.println();
	
	switch (currentState){
		case 1: 	// ON
			
			if (stateDiff > localParams.pulseDuration) // If we passed the time needed for this ramp section
			{
				currentState = 2;
				setPower(0);
				lastStateChange = now;
			}
			break;
			
		case 2: 	// OFF
			if (stateDiff > localParams.pulseDelay) // If we passed the time needed for this ramp section
			{
				if (currentRepetition < localParams.pulseRepetition)
				{
					currentRepetition++;
					setPower(localParams.pulsePower);
					currentState = 1;
				}
				else
				{
					currentRepetition = 1;
					setPower(0);					
					currentState = 3;
				}				
				lastStateChange = now;
			}
			break;
			
		case 3: 	// Idle
			if (stateDiff > localParams.sequenceDelay * MS_MULTIPLIER) // If we passed the time needed for this ramp section
			{
				setPower(localParams.pulsePower);
				currentState = 1;				
				lastStateChange = now;
			}
			break;
			
		default: break; // defaults to zero!
			
	}	
}


void VibrationMotor::beginSequence()
{	
	setPower(localParams.pulsePower);
	currentState = 1;
	currentRepetition = 1;
	lastStateChange = millis();
	isActive = true;	
}



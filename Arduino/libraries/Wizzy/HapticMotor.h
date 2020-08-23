#include <Adafruit_NeoPixel.h>
 
#pragma once

struct MotorParams {
  public:
	uint8_t powerLevel;
    unsigned long pulseTime, idleTime, numberOfPulses;	
	float filter;
};

class HapticMotor {
  public:
	static MotorParams wizzyClear; 
	static MotorParams wizzyA; 
	static MotorParams wizzyB; 
	static MotorParams wizzyC;
	const static unsigned DT=10;  // Milliseconds between iterations
	
    int motorId;
	int outputPin;
	bool isActive = false;
	unsigned long startTime=0;
	
	unsigned long lastToggle=0; // Last time the pulse changed from ON to OFF or vice-versa
	unsigned long toggleTime=0;
	float currentPower=0; 
	float desiredPower=0;	
	bool pulseState=0; // 1 means pulse is now ON, 0 means pulse is now OFF
	int currentPulse=1;
	
	MotorParams currentState;
	
    HapticMotor(int id, int pwmPin);    
    void turnOff(); 
    void setPower(int power);
	void iterateSequence();
	void beginSequence(char desiredState);
	void togglePulse(bool desiredState);
	void setChairState(char desiredState);
	static void defineChairState(char state, int _powerLevel, int _pulseTime, int _idleTime, int _numberOfPulses, float filer);
};

HapticMotor::HapticMotor(int id, int pwmPin) {
	outputPin = pwmPin;
	pinMode(pwmPin, OUTPUT);
	motorId = id; 
	turnOff();
}

static void HapticMotor::defineChairState(char state, int _powerLevel, int _pulseTime, int _idleTime, int _numberOfPulses, float _filter)
{
	MotorParams tempParams;
	tempParams.powerLevel = _powerLevel;
	tempParams.pulseTime = _pulseTime;
	tempParams.idleTime = _idleTime;
	tempParams.numberOfPulses = _numberOfPulses;
	tempParams.filter = _filter;
	
	switch (state){
		case 'O':		
			wizzyClear = tempParams;
			break;
		case 'A':
			wizzyA = tempParams;
			break;			
		case 'B':		
			wizzyB = tempParams;
			break;			
		case 'C':		
			wizzyC = tempParams;
			break;						
		default: 
			Serial.println("Warning: unknown state received.");
			break;
			
	}
}

void HapticMotor::setChairState(char desiredState){
	switch (desiredState){
		case 'O':		
			currentState = HapticMotor::wizzyClear;
			break;
		case 'A':		
			currentState = HapticMotor::wizzyA;
			break;			
		case 'B':		
			currentState = HapticMotor::wizzyB;
			break;			
		case 'C':		
			currentState = HapticMotor::wizzyC;
			break;						
		default: 
			Serial.println("Warning: unknown state received.");
			break;
			
	}
}

void HapticMotor::turnOff() {
	digitalWrite(outputPin, 0);
	currentPulse = 1;
	currentPower = 0;
	isActive = false;
}

void HapticMotor::setPower(int power) // Leaves color the same (even if 0) and scales it between 0-255
{
	analogWrite(outputPin, power);
}

void HapticMotor::beginSequence(char desiredState)
{
	setChairState(desiredState);
	startTime = millis();
	currentPulse = 1;
	togglePulse(1);
	isActive = true;
	//Serial.print("Started state ");
	//Serial.print(desiredState);
	//Serial.print(" on segment ");
	//Serial.println(sectionId);
}

void HapticMotor::togglePulse(bool desiredState)
{
	if (desiredState)
	{	
		currentPower = 0;
		desiredPower = currentState.powerLevel;
		toggleTime = currentState.pulseTime;		
	}
	else
	{
		currentPower = currentState.powerLevel;
		desiredPower = 0;
		toggleTime = currentState.idleTime;
		currentPulse++;
	}
	lastToggle = millis();	
	pulseState = desiredState;
}

void HapticMotor::iterateSequence()
{
	if (isActive)
	{
		unsigned long now = millis();
		currentPower = currentPower * currentState.filter + desiredPower * (1- currentState.filter);
		currentPower = constrain(currentPower, 0, 254);
		setPower(currentPower);
		//Serial.println(currentPower);
		if (now - lastToggle > toggleTime) togglePulse(!pulseState);
		if (currentPulse > currentState.numberOfPulses) turnOff();		
	}
}





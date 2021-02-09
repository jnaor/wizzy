#include <Adafruit_NeoPixel.h>
#include "BugSignalingMessage.h"
#include "WizzyDefinitions.h"
 
#pragma once

class LEDSection {
  public:
	Adafruit_NeoPixel* ledStrip;
    byte sectionId;
	bool isActive = false;
    uint8_t R;
    uint8_t G;
    uint8_t B;
	unsigned long lastIteration=0;
	unsigned long lastStateChange=0;
	float jumpValue;
	float currentBrightness;
	int currentState=0;
	LEDParams localParams;
	
    LEDSection(int id, Adafruit_NeoPixel* _ledStrip);    
    void turnOff(); 
    void setColor(uint8_t _R, uint8_t _G, uint8_t _B);
    void scalePixelColor(float _brightness);
	void iterateSequence();
	void applyAdvancement(int stateTime, int startValue, int endValue, int rampTime);
	void beginSequence();
};

LEDSection::LEDSection(int id, Adafruit_NeoPixel* _ledStrip) {
  sectionId = id; 
  ledStrip = _ledStrip;
}

void LEDSection::turnOff() {
  setColor(0,0,0);
  scalePixelColor(0);
  currentState = 0;
  isActive = false;
}

void LEDSection::setColor(uint8_t _R, uint8_t _G, uint8_t _B){
  R=_R;
  G=_G;
  B=_B;
  for(uint16_t i=4*sectionId; i<4*sectionId+4; i++) {
    ledStrip->setPixelColor(i, ledStrip->Color(_R,_G,_B) );
  }
}

void LEDSection::scalePixelColor(float _brightness) // Leaves color the same (even if 0) and scales it between 0-255
{
  uint8_t _R, _G, _B;
  float normalizeFactor = _brightness / 255.0; // Optimization
  _R = R * normalizeFactor;
  _G = G * normalizeFactor;
  _B = B * normalizeFactor;
  
  for(uint16_t i=4*sectionId; i<4*sectionId+4; i++) {
    ledStrip->setPixelColor(i, ledStrip->Color(_R,_G,_B) );
  }
}

void LEDSection::iterateSequence()
{	
	switch (currentState){
		case 1: 	//A			
			applyAdvancement(localParams.A, localParams.attackLevel, localParams.sustainLevel, localParams.D);
			break;			
		case 2: 	//D			
			applyAdvancement(localParams.D, localParams.sustainLevel, localParams.sustainLevel, localParams.S);
			break;			
		case 3: 	//S			
			applyAdvancement(localParams.S, localParams.sustainLevel, 0, localParams.R);
			break;			
		case 4: 	//R			
			applyAdvancement(localParams.R, 0, 0, localParams.sequenceDelay * MS_MULTIPLIER);
			break;			
		case 5: 	//Idle			
			applyAdvancement(localParams.sequenceDelay * MS_MULTIPLIER, 0, localParams.attackLevel, localParams.A);
			if (currentState > 5) currentState = 1;
			break;
			
		default: break; // defaults to zero!
			
	}	
}

void LEDSection::applyAdvancement(int stateTime, int startValue, int endValue, int rampTime)
{	
	// The first argument is for the current state, all 3 others are for calculations of the NEXT STATE.
	unsigned long now = millis();	
	unsigned long iterationDiff = now - lastIteration;
	unsigned long stateDiff = now - lastStateChange;
	
	//Serial.print(iterationDiff);
	//Serial.print(' ');
	//Serial.print(stateDiff);
	//Serial.print(' ');
	// Serial.print(currentState);
	// Serial.print(' ');
	// Serial.print(currentBrightness);
	// Serial.print(' ');
	// Serial.print(jumpValue);
	// Serial.println();
	
	scalePixelColor(constrain(currentBrightness, 0, 255));
	currentBrightness += jumpValue * iterationDiff; // pay for lost time between cycles
	lastIteration = now;
		
	if (stateDiff > stateTime) // If we passed the time needed for this ramp section
	{
		if (rampTime == 0) rampTime = 1; // Protect against division by zero
		jumpValue = float(endValue-startValue) / float(rampTime);
		currentBrightness = startValue;
		currentState++;
		lastStateChange = now;
	}
}

void LEDSection::beginSequence()
{
	// Adjust given colors to (almost) highest possible values, keeping the ratio between them:
	float normalizeFactor = 254.0 / max(max(max(localParams.red, localParams.green), localParams.blue), 1); // Find highest one, must be above zero
	int currentRed = localParams.red * normalizeFactor;
	int currentGreen = localParams.green * normalizeFactor;
	int currentBlue = localParams.blue * normalizeFactor;
	setColor(currentRed, currentGreen, currentBlue);
	int rampTime;
	
	if (localParams.A > 0) rampTime = localParams.A;
	else rampTime = 1; // Protection against division by zero
	
	jumpValue = float(localParams.attackLevel - 0) / float(rampTime);
	currentBrightness = 0;
	currentState = 1;
	lastStateChange = millis();
	lastIteration = lastStateChange;
	//Serial.println("seq start");
	isActive = true;
}



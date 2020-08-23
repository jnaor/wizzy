#include <Adafruit_NeoPixel.h>
 
#pragma once

struct LedParams {
  public:     
    uint8_t red, green, blue;
	uint8_t powerLevel;
    unsigned long pulseTime, idleTime, numberOfPulses;	
	float filter;
};

class LedSegment {
  public:
	static LedParams wizzyClear; 
	static LedParams wizzyA; 
	static LedParams wizzyB; 
	static LedParams wizzyC;
	const static unsigned DT=10;  // Milliseconds between iterations
	
	Adafruit_NeoPixel* ledStrip;
    int sectionId;
	bool isActive = false;
    uint8_t R;
    uint8_t G;
    uint8_t B;
	unsigned long startTime=0;
	
	unsigned long lastToggle=0; // Last time the pulse changed from ON to OFF or vice-versa
	unsigned long toggleTime=0;
	float currentBrightness=0; 
	float desiredBrightness=0;	
	bool pulseState=0; // 1 means pulse is now ON, 0 means pulse is now OFF
	int currentPulse=1;
	
	LedParams currentState;
	
    LedSegment(int id, Adafruit_NeoPixel* _ledStrip);    
    void turnOff(); 
    void setColor(uint8_t _R, uint8_t _G, uint8_t _B);
    void setBrightness(float _brightness);
	void iterateSequence();
	void beginSequence(char desiredState);
	void togglePulse(bool desiredState);
	void setChairState(char desiredState);
	static void defineChairState(char state, int _red, int _green, int _blue, int _powerLevel, int _pulseTime, int _idleTime, int _numberOfPulses, float filer);
};

LedSegment::LedSegment(int id, Adafruit_NeoPixel* _ledStrip) {
  sectionId = id; 
  ledStrip = _ledStrip;
  turnOff();
}

static void LedSegment::defineChairState(char state, int _red, int _green, int _blue, int _powerLevel, int _pulseTime, int _idleTime, int _numberOfPulses, float _filter)
{
	LedParams tempParams;
	tempParams.red = _red;
	tempParams.green = _green;
	tempParams.blue = _blue;
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

void LedSegment::setChairState(char desiredState){
	switch (desiredState){
		case 'O':		
			currentState = LedSegment::wizzyClear;
			break;
		case 'A':		
			currentState = LedSegment::wizzyA;
			break;			
		case 'B':		
			currentState = LedSegment::wizzyB;
			break;			
		case 'C':		
			currentState = LedSegment::wizzyC;
			break;						
		default: 
			Serial.println("Warning: unknown state received.");
			break;
			
	}
}

void LedSegment::turnOff() {
  setColor(0,0,0);
  setBrightness(0);
  currentPulse = 1;
  currentBrightness = 0;
  isActive = false;
}

void LedSegment::setColor(uint8_t _R, uint8_t _G, uint8_t _B){
  R=_R;
  G=_G;
  B=_B;
  for(uint16_t i=4*sectionId; i<4*sectionId+4; i++) {
    ledStrip->setPixelColor(i, ledStrip->Color(_R,_G,_B) );
  }
}

void LedSegment::setBrightness(float _brightness) // Leaves color the same (even if 0) and scales it between 0-255
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

void LedSegment::beginSequence(char desiredState)
{
	setChairState(desiredState);
	// Adjust given colors to (almost) highest possible values, keeping the ratio between them:
	float normalizeFactor = 254.0 / max(max(max(currentState.red, currentState.green), currentState.blue), 1); // Find highest one, must be above zero
	int currentRed = currentState.red * normalizeFactor;
	int currentGreen = currentState.green * normalizeFactor;
	int currentBlue = currentState.blue * normalizeFactor;
	setColor(currentRed, currentGreen, currentBlue);
	startTime = millis();
	currentPulse = 1;
	togglePulse(1);
	isActive = true;
	//Serial.print("Started state ");
	//Serial.print(desiredState);
	//Serial.print(" on segment ");
	//Serial.println(sectionId);
}

void LedSegment::togglePulse(bool desiredState)
{
	if (desiredState)
	{	
		currentBrightness = 0;
		desiredBrightness = currentState.powerLevel;
		toggleTime = currentState.pulseTime;		
	}
	else
	{
		currentBrightness = currentState.powerLevel;
		desiredBrightness = 0;
		toggleTime = currentState.idleTime;
		currentPulse++;
	}
	lastToggle = millis();	
	pulseState = desiredState;
}

void LedSegment::iterateSequence()
{
	if (isActive)
	{
		unsigned long now = millis();
		currentBrightness = currentBrightness * currentState.filter + desiredBrightness * (1- currentState.filter);
		currentBrightness = constrain(currentBrightness, 0, 254);
		setBrightness(currentBrightness);
		if (now - lastToggle > toggleTime) togglePulse(!pulseState);
		if (currentPulse > currentState.numberOfPulses) turnOff();

		//Serial.print(currentBrightness);
		//Serial.print("  ");
		//Serial.print(currentPulse);
		//Serial.print("  ");
		//Serial.println(now - lastToggle);
		//Serial.print("  ");
		//Serial.println(toggleTime);		
	}
}





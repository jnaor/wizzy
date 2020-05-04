 
#pragma once

struct LEDParams {
  public:     
	uint8_t flags;
    uint8_t red, green, blue;
    uint8_t A, D, S, R;
    uint8_t attackLevel, sustainLevel;
	uint8_t sequenceDelay;    
};

struct MotorParams {
  public:    
	uint8_t flags;
	uint8_t pulseDuration;
	uint8_t pulsePower;
	uint8_t pulseRepetition;
	uint8_t pulseDelay;
	uint8_t sequenceDelay;
};
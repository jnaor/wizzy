#pragma once
#include "WizzyDefinitions.h"

class BugSignalingMessage {

  public:    
	bool newDataReceived;
	int receiveDelay;
	
	uint8_t sections;
	LEDParams ledParams;
	
	uint8_t motors;
	MotorParams motorParams;
	
	BugSignalingMessage(int baudrate);
    int receiveMessage();
    bool waitForInput(int bytesToWait);
    void printLedMembers();
	void printMotorMembers();
    void debugValues();
    uint8_t readWithDelay(unsigned int milliseconds);
};

BugSignalingMessage::BugSignalingMessage(int baudrate)
{
  receiveDelay = int(1.0 / float(baudrate) * 8e6);
}


bool BugSignalingMessage::waitForInput(int bytesToWait)
{
  if (!Serial.available())
  {
    delayMicroseconds(receiveDelay * bytesToWait); // The time it takes 1 byte to arrive on the given baud
  }
  return Serial.available();
}

uint8_t BugSignalingMessage::readWithDelay(unsigned int milliseconds)
{
  delay(milliseconds);
  uint8_t messageInput = Serial.read();
  //Serial.println(messageInput);
  return (messageInput);
}

int BugSignalingMessage::receiveMessage()
{
  while (Serial.available())
  {
	uint8_t messageInput = readWithDelay(1);
    if (messageInput == 'A')
    {
      if (readWithDelay(1) == 'A') // Double 'AA' header = LED command
      {
		sections = readWithDelay(1);
        ledParams.red = readWithDelay(1);
        ledParams.green = readWithDelay(1);
        ledParams.blue = readWithDelay(1);
        ledParams.A = readWithDelay(1);
        ledParams.D = readWithDelay(1);
        ledParams.S = readWithDelay(1);
        ledParams.R = readWithDelay(1);
        ledParams.attackLevel = readWithDelay(1);
        ledParams.sustainLevel = readWithDelay(1);
		ledParams.sequenceDelay = readWithDelay(1);
        ledParams.flags = readWithDelay(1);
        return 1;
      }
    }
	if (messageInput == 'B')
    {
      if (readWithDelay(1) == 'B') // Double 'BB' header = motor command
      {
		motors = readWithDelay(1);
		motorParams.pulsePower = readWithDelay(1);
		motorParams.pulseDuration = readWithDelay(1);		
		motorParams.pulseRepetition = readWithDelay(1);
		motorParams.pulseDelay = readWithDelay(1);
		motorParams.sequenceDelay = readWithDelay(1);
		motorParams.flags = readWithDelay(1);
        return 2;
      }
    }
  }
  return 0;
}

void BugSignalingMessage::debugValues()
{
	sections = 1; //5; // Means: sections 0 and 2: 00000101
	ledParams.red = 10;
	ledParams.green = 1;
	ledParams.blue = 5;
	ledParams.A = 150;
	ledParams.D = 150;
	ledParams.S = 200;
	ledParams.R = 150;
	ledParams.attackLevel = 200;
	ledParams.sustainLevel = 50;
	ledParams.sequenceDelay = 255;
	ledParams.flags = 0;
	
	motors = 3; // Means: motors 0 and 1: 00000011
	motorParams.pulsePower = 200;
	motorParams.pulseDuration = 200;	
	motorParams.pulseRepetition = 3;
	motorParams.pulseDelay = 200;
	motorParams.sequenceDelay = 255;
	motorParams.flags = 0;
}

void BugSignalingMessage::printLedMembers()
{
  Serial.println(sections);
  Serial.println(ledParams.red);
  Serial.println(ledParams.green);
  Serial.println(ledParams.blue);
  Serial.println(ledParams.A);
  Serial.println(ledParams.D);
  Serial.println(ledParams.S);
  Serial.println(ledParams.R);
  Serial.println(ledParams.attackLevel);
  Serial.println(ledParams.sustainLevel);
  Serial.println(ledParams.sequenceDelay);
  Serial.println(ledParams.flags);
}

void BugSignalingMessage::printMotorMembers()
{
  Serial.println(motors);
  Serial.println(motorParams.pulsePower);
  Serial.println(motorParams.pulseDuration);
  Serial.println(motorParams.pulseRepetition);
  Serial.println(motorParams.pulseDelay);
  Serial.println(motorParams.sequenceDelay);
  Serial.println(motorParams.flags);
}

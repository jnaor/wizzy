#pragma once

class StateMessage {

  public:    
	bool newDataReceived;
	int receiveDelay;
	
	uint8_t desiredSection;
	char desiredState;
	
	StateMessage(int baudrate);
    int receiveMessage();
    bool waitForInput(int bytesToWait);
    void debugValues();
	void setState(char desiredState);
    uint8_t readWithDelay(unsigned int milliseconds);
};

StateMessage::StateMessage(int baudrate)
{
  receiveDelay = int(1.0 / float(baudrate) * 8e6);
}


bool StateMessage::waitForInput(int bytesToWait)
{
  if (!Serial.available())
  {
    delayMicroseconds(receiveDelay * bytesToWait); // The time it takes 1 byte to arrive on the given baud
  }
  return Serial.available();
}

uint8_t StateMessage::readWithDelay(unsigned int milliseconds)
{
  delay(milliseconds);
  uint8_t messageInput = Serial.read();
  return (messageInput);
}

int StateMessage::receiveMessage()
{
  while (Serial.available()>0)  // Message format is "HH<section><state>TT"
  {
    if (readWithDelay(1) == 'H')
      if (readWithDelay(1) == 'H')
      {
		desiredSection = readWithDelay(1);
        desiredState = readWithDelay(1);
		if (readWithDelay(1) == 'T') 
			if (readWithDelay(1) == 'T')
				return 1;
      }
	delay(1);
  }
  return 0;
}

void StateMessage::debugValues()
{
	desiredSection = 2;
	desiredState = 'A';
}


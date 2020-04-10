#include <Adafruit_NeoPixel.h>
#include "BugSignalingMessage.h"

#ifdef __AVR__
  #include <avr/power.h>
#endif

#define SERIAL_BAUD 9600
#define LED_PIN 6
#define NUM_LEDS 24
#define MAX_BRIGHTNESS 255
#define MS_MULTIPLIER 1

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB);

LEDSection ledSections[6] ={LEDSection(0, &strip), LEDSection(1, &strip), LEDSection(2, &strip), LEDSection(3, &strip), LEDSection(4, &strip), LEDSection(5, &strip)}; 
BugSignalingMessage msg=BugSignalingMessage(SERIAL_BAUD); 

void initiateSequence()
{
  int currentSection;
  // Adjust given colors to (almost) highest possible values, keeping the ratio between them:
  float normalizeFactor = max(max(msg.red, msg.green), msg.blue); // Find highest one
  int currentRed = int(float(msg.red)*250.0/normalizeFactor);
  int currentGreen = int(float(msg.green)*250.0/normalizeFactor);
  int currentBlue = int(float(msg.blue)*250.0/normalizeFactor);
  
  for (currentSection=0; currentSection<6; currentSection++)
  {
    // If the bit was set to true, update the colors of the corresponding section, otherwise make sure it is off:
    if (isIndexValid(msg.sections, currentSection))
    {
      ledSections[currentSection].setColor(currentRed, currentGreen, currentBlue);
    }      
    else
    {
      ledSections[currentSection].turnOff();
    }      
  } 
  
  // ##################### Beginning of sequence ####################
  // Raising from 0 to desired brightness in time A:
  implementRamp(20, msg.attackLevel, msg.A);
  //if (msg.receiveMessage()) return; // So that we can stop if we got a new message
  
  // Coming down from desired brightness to sustain brightness in time D:
  implementRamp(msg.attackLevel, msg.sustainLevel, msg.D);
  //if (msg.receiveMessage()) return;

  // Holding on sustain brightness for desired amount of time (S is a level, not time!):
  implementRamp(msg.sustainLevel, msg.sustainLevel, msg.S);
  //if (msg.receiveMessage()) return;

  // Coming down from sustain brightness to 0 in time R:
  implementRamp(msg.sustainLevel, 0, msg.R);

  // Turn off all sections:
  for (currentSection=0; currentSection<6; currentSection++)
  {
    ledSections[currentSection].turnOff();
  }
  
}

void implementRamp(int startValue, int endValue, int rampTime) // NEEDS A CUSTOM SET-BRIGHTNESS FUNCTION!
{
  int i, currentSection;
  float currentBrightness = startValue;
  float jumpValue = (endValue-startValue) / float(rampTime);
  for (i=0; i<rampTime; i++)
  {
    for (currentSection=0; currentSection<6; currentSection++)
    {
      if (isIndexValid(msg.sections, currentSection))
      {
        ledSections[currentSection].scalePixelColor(currentBrightness);
      }
    }
    strip.setBrightness(constrain(currentBrightness, 0, 255));
    strip.show();
    currentBrightness += jumpValue;
    delay(MS_MULTIPLIER);
  }
}

bool isIndexValid(byte sections, uint8_t index)
{
  if (1 << index & sections)
    return true;
  else
    return false;
}

void setup() {    
  Serial.begin(SERIAL_BAUD); 
  strip.setBrightness(MAX_BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  if (Serial.available())
  {
    if (msg.receiveMessage())
    {
      Serial.print("OK");
    }
    else
    {
      Serial.print("Wrong format");
    }
  }  
//  msg.debugValues();
  initiateSequence();

  delay(1000);
}

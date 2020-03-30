#include <Adafruit_NeoPixel.h>

#define SERIAL_BAUD 9600
#define LED_PIN 3
#define NUM_LEDS 24
#define MAX_BRIGHTNESS 255
#define MS_MULTIPLIER 4
#define TEST_MOTOR_PIN 10

#include "LEDSection.h"
#include "VibrationMotor.h"
#include "BugSignalingMessage.h"

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB);

LEDSection ledSections[6] = {LEDSection(0, &strip), LEDSection(1, &strip), LEDSection(2, &strip), LEDSection(3, &strip), LEDSection(4, &strip), LEDSection(5, &strip)}; 
VibrationMotor vibMotors[6] = {VibrationMotor(0, 10), VibrationMotor(0, 9), VibrationMotor(0, 11), VibrationMotor(0, 11), VibrationMotor(0, 11), VibrationMotor(0, 11)};
BugSignalingMessage msg=BugSignalingMessage(SERIAL_BAUD);
int messageType = 0;
int currentIndex;

bool isIndexValid(byte instances, uint8_t index)
{
  if (1 << index & instances)
    return true;
  else
    return false;
}

void setup() {    
  Serial.begin(SERIAL_BAUD); 
  strip.setBrightness(MAX_BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  pinMode(TEST_MOTOR_PIN, OUTPUT);
  analogWrite(TEST_MOTOR_PIN, 200);
  delay(200);
  analogWrite(TEST_MOTOR_PIN, 0);

  //msg.debugValues();
  
//  for (currentIndex=0; currentIndex<6; currentIndex++)
//  {
//    if (isIndexValid(msg.sections, currentIndex))
//    {
//      ledSections[currentIndex].localParams = msg.ledParams;
//      ledSections[currentIndex].beginSequence();
//    }
//  }
//
//  for (currentIndex=0; currentIndex<6; currentIndex++)
//  {
//    if (isIndexValid(msg.sections, currentIndex)) // Add something for turnOff()
//    {
//      vibMotors[currentIndex].localParams = msg.motorParams;
//      vibMotors[currentIndex].beginSequence();
//    }
//  }
}

void loop() {
      
  if (Serial.available())
  {
    messageType = msg.receiveMessage();
    Serial.print(messageType);
    Serial.print("! ");
    if (messageType == 1)
    {
      for (currentIndex=0; currentIndex<6; currentIndex++)
      {
        if (isIndexValid(msg.sections, currentIndex)) // Add something for turnOff()
        {
          ledSections[currentIndex].localParams = msg.ledParams;
          ledSections[currentIndex].beginSequence();
        }
      }
      Serial.print("OK:1");
    }
    
    if (messageType == 2)
    {
      for (currentIndex=0; currentIndex<6; currentIndex++)
      {
        if (isIndexValid(msg.motors, currentIndex)) // Add something for turnOff()
        {
          vibMotors[currentIndex].localParams = msg.motorParams;
          vibMotors[currentIndex].beginSequence();
        }
      }
      Serial.print("OK:2 ");
    }
    
    if (messageType == 0)
    {
      Serial.print("Wrong format ");
    }
  }  

  for (currentIndex=0; currentIndex<6; currentIndex++)
  {
    if (ledSections[currentIndex].isActive) 
    {
      ledSections[currentIndex].iterateSequence();      
    }
    if (vibMotors[currentIndex].isActive) 
    {
      vibMotors[currentIndex].iterateSequence();      
    }
  }
  
  strip.show();  
 
}

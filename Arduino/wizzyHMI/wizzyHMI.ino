#include <Adafruit_NeoPixel.h>

#define SERIAL_BAUD 9600
#define LED_PIN 3
#define NUM_LEDS 24
#define MAX_BRIGHTNESS 255
#define MS_MULTIPLIER 4

#include "LedSegment.h"
#include "HapticMotor.h"
#include "StateMessage.h"

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB);

LedSegment ledSections[6] = {LedSegment(0, &strip), LedSegment(1, &strip), LedSegment(2, &strip), LedSegment(3, &strip), LedSegment(4, &strip), LedSegment(5, &strip)}; 
HapticMotor hapticMotors = HapticMotor(0, 5);  // If multiple motors, each should have its own PWM-enabled pin!
StateMessage msg = StateMessage(SERIAL_BAUD);

LedParams LedSegment::wizzyClear = LedParams();
LedParams LedSegment::wizzyA = LedParams();
LedParams LedSegment::wizzyB = LedParams();
LedParams LedSegment::wizzyC = LedParams();

MotorParams HapticMotor::wizzyClear = MotorParams();
MotorParams HapticMotor::wizzyA = MotorParams();
MotorParams HapticMotor::wizzyB = MotorParams();
MotorParams HapticMotor::wizzyC = MotorParams();

void blinkBuiltInLed()
{
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
  delay(200);
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
}

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(SERIAL_BAUD); 
  strip.setBrightness(MAX_BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  //                    Order:( S ,  R ,  G ,  B , pwr, t_p, t_i, n, fltr)
  LedSegment::defineChairState('O', 000, 000, 000, 000, 001, 001, 0, 0.5);
  LedSegment::defineChairState('A', 50, 250, 000, 80, 500, 001, 1, 0.9);
  LedSegment::defineChairState('B', 250, 100, 000, 120, 350, 150, 2, 0.9);
  LedSegment::defineChairState('C', 230, 000, 010, 200, 230, 100, 5, 0.9);

  //                    Order:( S ,  pwr, t_p, t_i, n, fltr)
  HapticMotor::defineChairState('O', 000, 001, 001, 0, 0.5);
  HapticMotor::defineChairState('A', 70, 500, 001, 1, 0.9);
  HapticMotor::defineChairState('B', 100, 350, 150, 2, 0.9);
  HapticMotor::defineChairState('C', 130, 230, 100, 5, 0.9);
}

void loop() {      
  if (Serial.available()>0){
    //Serial.println(Serial.available());
    bool isMessageValid = msg.receiveMessage();  // HH1CTT
    uint8_t terminator = Serial.peek();
    for (int i=0; i<2; i++)
      if (terminator == 10 or terminator == 13)  // Remove LF/CR termination from message
        uint8_t dump = Serial.read();
        terminator = Serial.peek(); 
        
    if (isMessageValid)
    {
//      Serial.print(msg.desiredSection);
//      Serial.print("   ");
//      Serial.print(msg.desiredState);
//      Serial.print("   ");
      Serial.println("OK");
      //blinkBuiltInLed();
      for (int idx=0; idx<6; idx++) 
        if (ledSections[idx].isActive) ledSections[idx].turnOff();
      msg.desiredSection -= 48;
      if (msg.desiredSection >= 0 and msg.desiredSection <=6)
        ledSections[msg.desiredSection].beginSequence(msg.desiredState);
      hapticMotors.beginSequence(msg.desiredState);
    }    
    else
    {
      Serial.println("Fail");
    }
  }
  hapticMotors.iterateSequence();
  for (int idx=0; idx<6; idx++)
    ledSections[idx].iterateSequence();
  strip.show(); 
  delay(1);
}

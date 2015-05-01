#include "VisionDC.h"

void VisionDC::init(int fwPin, int bwPin)
{
  forwardPin = fwPin;
  backwardPin = bwPin;
  
  pinMode(forwardPin, OUTPUT);
  pinMode(backwardPin, OUTPUT);  
  
  digitalWrite(forwardPin, LOW);  
  digitalWrite(backwardPin, LOW);
}

void VisionDC::moveForward(unsigned char pwm)
{
  analogWrite(forwardPin, pwm);  
  digitalWrite(backwardPin, LOW);
}

void VisionDC::moveBackward(unsigned char pwm)
{
  digitalWrite(forwardPin, LOW);  
  analogWrite(backwardPin, pwm);
}

void VisionDC::stopMotor()
{
  digitalWrite(forwardPin, LOW);  
  digitalWrite(backwardPin, LOW);
}


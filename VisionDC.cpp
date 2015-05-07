#include "VisionDC.h"

void VisionDC::init(int fwPin/*, int bwPin*/)
{
  forwardPin = fwPin;
  
  pinMode(forwardPin, OUTPUT);
  
  digitalWrite(forwardPin, LOW);  
}

void VisionDC::moveForward(unsigned char pwm)
{
  analogWrite(forwardPin, pwm);  
  isOn = true;
}
void VisionDC::stopMotor()
{
  digitalWrite(forwardPin, LOW);  
  isOn = false;
}


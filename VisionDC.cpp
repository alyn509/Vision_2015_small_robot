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

void VisionDC::moveForward()
{
  digitalWrite(forwardPin, HIGH);  
  digitalWrite(backwardPin, LOW);
}

void VisionDC::moveBackward()
{
  digitalWrite(forwardPin, LOW);  
  digitalWrite(backwardPin, HIGH);
}

void VisionDC::stopMotor()
{
  digitalWrite(forwardPin, LOW);  
  digitalWrite(backwardPin, LOW);
}


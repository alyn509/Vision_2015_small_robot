#include "VisionEncoders.h"
#include "pins.h"
#include <elapsedMillis.h>

void VisionEncoders::init(int pinStep) 
{ 
   stepPin = pinStep;
   pinMode (stepPin,INPUT);
} 

long VisionEncoders::getPosition(bool dir) 
{ 
   int currentState = digitalRead(stepPin);
   if ((lastState == LOW) && (currentState == HIGH)) 
   {
     if (dir == LOW) 
       currentPossition--;
     else
       currentPossition++;
   } 
   lastState = currentState;
   return currentPossition;
} 

#include <elapsedMillis.h>
#include <Stepper.h>
#include <LiquidCrystal.h>
#include <Servo.h> 
#include "VisionBase.h"
#include "VisionDevices.h"
#include "VisionState.h"
#include "pins.h"
#include "constants.h"

#define NINETYSECONDS 39000L

VisionBase base;
VisionDevices devices;
elapsedMillis timeUpTimer;
boolean stoppedEverything = true; 

VisionState state;
int team_color;
int tactic;
float distanceToDo = 0;

void setup()
{ 
  while(digitalRead(startButtonPin) == 1);
  delay(2000);
  base.init();
}

void loop()
{ 
  base.doLoop();
}


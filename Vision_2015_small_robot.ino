#include <elapsedMillis.h>
#include <Stepper.h>
#include <LiquidCrystal.h>
#include <Servo.h> 
#include "VisionBase.h"
#include "VisionDevices.h"
#include "VisionState.h"
#include "VisionSensor.h"
#include "pins.h"
#include "constants.h"

#define NINETYSECONDS 39000L

VisionBase base;
VisionDevices devices;
elapsedMillis timeUpTimer;
boolean stoppedEverything = true; 

VisionSensor startButton;
int team_color;

void setup()
{ 
  startButton.init(startButtonPin);
  while(!startButton.detect());
  delay(2000);
  base.init();
}

void loop()
{ 
  base.doLoop();
}


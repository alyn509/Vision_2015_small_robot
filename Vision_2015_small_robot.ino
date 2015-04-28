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
  //Serial.begin(115200);
  timeUpTimer = 0;
  base.init();
  //devices.init();
  team_color = base.oppositeSide ? 20 : 0;
 // team_color = (team_color == HOMOLOGATION) ? 0 : team_color;
  state.wait(100, tactic + team_color);
  stoppedEverything = false;
}

void loop()
{ 
  /*while(1)
  {
    Serial.print(base.frontDetected());
    Serial.print("             ");
    Serial.println(base.backDetected());
  }*/
  switch (state)
  {  
    default:
      state.doLoop();
  }
  if(!stoppedEverything)
  {
    base.checkObstructions();
    //base.doLoop();
  }
}


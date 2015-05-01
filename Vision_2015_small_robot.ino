#include <SoftwareServo.h>
#include <elapsedMillis.h>
#include "VisionBase.h"
#include "VisionDevices.h"
#include "VisionState.h"
#include "VisionSensor.h"
#include "pins.h"
#include "constants.h"

//#define NINETYSECONDS 39000L

VisionBase base;
VisionDevices devices;
VisionState state;

//elapsedMillis timeUpTimer;
//boolean stoppedEverything = true; 

//VisionSensor startButton;
//int team_color;

void setup()
{ 
  //startButton.initPin(startButtonPin);
  //startButton.setAsPullup();
  //while(!startButton.detect());
  delay(2000);
  base.init();
}

void loop()
{
  base.doLoop();
  
  switch (state)
  {  
    case 0:      
      base.update();
      SoftwareServo::refresh();
      state.waitMicros(1000, 0);
      break;
    default:
      state.doLoop();
  }
  
}


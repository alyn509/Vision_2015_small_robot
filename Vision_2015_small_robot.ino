#include <SoftwareServo.h>
#include <elapsedMillis.h>
#include "VisionBase.h"
#include "VisionDevices.h"
#include "VisionState.h"
#include "VisionSensor.h"
#include "pins.h"
#include "constants.h"
#include <HMC5883L.h>
#include <Wire.h>

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
  Serial.begin(115200);
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
      state.waitMicros(10000, 0);
      break;
    default:
      state.doLoop();
  }
  
}


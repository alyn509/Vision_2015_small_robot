#include <Servo.h>
#include <elapsedMillis.h>
#include "VisionBase.h"
#include "VisionState.h"
#include "VisionSensor.h"
#include "pins.h"
#include "constants.h"

#define STOP_NOW -1
#define NINETYSECONDS 89000L

VisionBase base;
VisionState state;

elapsedMillis timeUpTimer, enemyTimer;
boolean stoppedEverything; 

VisionSensor startButton;

void setup()
{ 
  startButton.initPin(startButtonPin);
  startButton.setAsPullup();
//  while(!startButton.detect());
  delay(2000);
  timeUpTimer = 0;
  stoppedEverything = false;
  Serial.begin(115200);
  base.init();
}

void loop()
{
  switch (state)
  {  
    case 0:      
      base.update();
      state.waitMicros(10000, 0);
      break;
    default:
      state.doLoop();
  }   
  if(!stoppedEverything)
  {
    checkForObstacle();
    base.doLoop();
  }
  testIfTimeUp();
}

void testIfTimeUp()
{
  if(timeUpTimer >= NINETYSECONDS)
    timeIsUpStopEverything();  
}

void timeIsUpStopEverything()
{
  stoppedEverything = true;
  state = STATE_STOP;
  base.stopNow();
}

void checkForObstacle()
{
  if(!base.isStopped)
  {
    if(base.sensorDetection() == true ) 
      enemyTimer = 0;
    if(base.sensorDetection() == true && !base.isPaused && !base.ignoredSensors)   
      base.pause();
    else if(base.sensorDetection() == false && base.isPaused && enemyTimer > 500L)
      base.unpause();
  }
}

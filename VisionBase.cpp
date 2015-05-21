#include "VisionBase.h"

void VisionBase::init()
{
  color.initPin(colorPin);
  color.setAsPullup();
  
  frontLeft.initPin(frontLeftSensorPin);
  
  leftMotor.init(leftMotorFw);
  rightMotor.init(rightMotorFw);
  side = color.detect();
}

void VisionBase::moveForward(int pwmv)
{
  if(!newMovement || isResuming)
  {  
    directionMovement = FRONT;
    leftMotor.moveForward(pwmv);
    rightMotor.moveForward(pwmv);
    isResuming = false;
    pwmValue = pwmv;
  }
}
void VisionBase::turnLeft(int pwmv)
{  
  if(!newMovement || isResuming)
  {  
    directionMovement = LEFT;
    rightMotor.moveForward(pwmv);
    isResuming = false;
    pwmValue = pwmv;
  }
}

void VisionBase::turnRight(int pwmv)
{    
  if(!newMovement || isResuming)
  {  
    directionMovement = RIGHT;
    leftMotor.moveForward(pwmv);
    isResuming = false;
    pwmValue = pwmv;
  }
}
bool VisionBase::leftMotorDir()
{
  return (directionMovement / 2) % 2;
}

bool VisionBase::rightMotorDir()
{  
  return directionMovement % 2;
}
float VisionBase::cmToSteps(float value)
{
  return (1.0 * value * encoderResolution) / (PI * wheelDiameter);
}
float VisionBase::angleToSteps(float value)
{
  return (1.0 * value * distanceBetweenWheels * PI) / 360.0;
}
void VisionBase::doDistanceInCM(int dist, int nextState)
{
}
void VisionBase::doAngleRotation(int dist, int nextState)
{
}

void VisionBase::stopNow()
{      
  rightMotor.stopMotor();      
  leftMotor.stopMotor();  
  isStopped = true;
}
void VisionBase::doLoop()
{  
  switch (state)
  { 
    case 0: stopNow();moveForward(50);state.wait(1000, STATE_NEXT);break;
    case 1: stopNow();turnLeft(50);state.wait(1000, STATE_NEXT);break;
    case 2: stopNow();moveForward(50);state.wait(1000, STATE_NEXT);break;
  }
}

int integral, last;
void VisionBase::update()
{
}

boolean VisionBase::sensorDetection()
{
  return frontLeft.detect();
}

void VisionBase::pause()
{ 
  isPaused = true;
  stateBeforePause = state;
  state = PAUSED;
  rightMotor.stopMotor();      
  leftMotor.stopMotor();
}
void VisionBase::unpause()
{
  state = stateBeforePause;
  isPaused = false;
  isResuming = true;
}

void VisionBase::releaseCarpets()
{
  if(!carpetClaw.attached())
  {
    carpetClaw.attach(carpetDropClawPin);
    carpetClaw.write(60);
  }
}


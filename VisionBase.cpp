#include "VisionBase.h"

void VisionBase::init()
{
  color.initPin(colorPin);
  color.setAsPullup();
  
  frontLeft.initPin(frontLeftSensorPin);
  frontRight.initPin(frontRightSensorPin);
  
  leftMotor.init(leftMotorFw);
  rightMotor.init(rightMotorFw);
  
  leftEncoder.init(leftEncoderStepPin);
  rightEncoder.init(rightEncoderStepPin);
  
  moveBeacon(95);
  /*
  if(color.detect())
    state = YELLOW_SIDE;
  else*/
    state = YELLOW_SIDE;
}

void VisionBase::moveForward(int pwmv)
{  
  if(!newMovement || isResuming)
  {  
    directionMovement = FRONT;
    leftMotor.moveForward(pwmv);
    rightMotor.moveForward(pwmv);
    if(!isResuming)
    {
      leftEncoder.resetPosition();
      rightEncoder.resetPosition();
    }
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
    if(!isResuming)
    {
      leftEncoder.resetPosition();
      rightEncoder.resetPosition();
    }
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
    if(!isResuming)
    {
      leftEncoder.resetPosition();
      rightEncoder.resetPosition();
    }
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
  float steps = cmToSteps(dist);
  if(!newMovement)
  {
    newMovement = true;
    lastPositionLeft = leftEncoder.getPosition();
    lastPositionRight = rightEncoder.getPosition();
  }
  else
  {
    if(leftEncoder.getPosition() - lastPositionLeft >= steps)
      leftMotor.stopMotor();
    if(rightEncoder.getPosition() - lastPositionRight >= steps)
      rightMotor.stopMotor();
    if(!leftMotor.isOn && !rightMotor.isOn && !isPaused)
    {
      newMovement = false;
      state = nextState;
    }
  }
}
void VisionBase::doAngleRotation(int dist, int nextState)
{
  float steps = cmToSteps(2 * angleToSteps(dist));
  if(!newMovement)
  {
    newMovement = true;
    lastPositionLeft = leftEncoder.getPosition();
    lastPositionRight = rightEncoder.getPosition();
  }
  else
  {
    if(leftEncoder.getPosition() - lastPositionLeft >= steps)
      leftMotor.stopMotor();
    if(rightEncoder.getPosition() - lastPositionRight >= steps)
      rightMotor.stopMotor();
    if(!leftMotor.isOn && !rightMotor.isOn && !isPaused)
    {
      newMovement = false;
      state = nextState;
    }
  }
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
/************************************** YELLOW SIDE**************************************/      
    case 0:    
      moveForward(50);
      doDistanceInCM(75,1);
      break;
    case 1:    
      turnLeft(80);
      doAngleRotation(88,CLIMB_STAIRS);
      break;
/************************************** GREEN SIDE**************************************/      
    case 100:    
      moveForward(50);
      doDistanceInCM(75,101);
      break;
    case 101:    
      turnRight(80);
      doAngleRotation(92,CLIMB_STAIRS);
      break;
      
/************************************** CLIMB STAIRS**************************************/      
    case CLIMB_STAIRS:   
      moveForward(50);
      //lowerBeacon();
      doDistanceInCM(20,51);
      break;
    case 51:   
      moveForward(75);
      lowerBeacon();
      doDistanceInCM(36,52);
      break;
    case 52:   
      moveForward(70);
      lowerBeacon();
      doDistanceInCM(10,53);
      break;
    case 53:   
      moveForward(40);
      resumeBeacon();
      releaseCarpets();
      doDistanceInCM(15,OVER);
      break;
/************************************ HOMOLOGATION **************************************/      
      
    case HOMOLOGATION:    
      moveForward(80);
      doDistanceInCM(30,HOMOLOGATION + 1);
      break;
    case HOMOLOGATION + 1:    
      turnRight(80);
      doAngleRotation(90,HOMOLOGATION);
      break;      
    case PAUSED:
      break;
    case OVER: 
      moveBeacon(95);   
      stopNow();
      break;
    default:
      state.doLoop();
  }
  leftEncoder.updatePosition();
  rightEncoder.updatePosition();
}

int currentBeaconPos = 0;
int threshold = 75;
elapsedMillis beaconTimer;
void VisionBase::lowerBeacon()
{
  if(currentBeaconPos == 0)
    beaconTimer = 0;
  if(beaconTimer % 55 == 0 && currentBeaconPos < threshold)
  {
    currentBeaconPos++;
    moveBeacon(95 - currentBeaconPos);
  }
  
}
void VisionBase::resumeBeacon()
{
  if(currentBeaconPos == threshold)
    beaconTimer = 0;
  if(beaconTimer % 30 == 0 && currentBeaconPos > 0)
  {
    currentBeaconPos--;
    moveBeacon(95 - currentBeaconPos);
  }  
}

int integral, last;
void VisionBase::update()
{
/*  Serial.print(" L: ");
  Serial.print(leftEncoder.getPosition());
  Serial.print(" R: ");
  Serial.print(rightEncoder.getPosition());
*/
  if(directionMovement == FRONT)
  {
    int threshold = pwmValue - 10;
    int difference = leftEncoder.getPosition() - rightEncoder.getPosition();
    int deriv = difference - last;
    last = difference;
    integral += difference;
    int turn = 2.0 * difference + 0.0 * integral + 0.0 * deriv;
    if (turn > threshold) turn = threshold;
    if (turn < -threshold) turn = -threshold;
    if (leftMotor.isOn)
      leftMotor.moveForward(pwmValue - turn);
    if (rightMotor.isOn)
      rightMotor.moveForward(pwmValue + turn);
/*    Serial.print(" E: ");
    Serial.print(difference);
    Serial.print(" I: ");
    Serial.print(integral);
    Serial.print(" D: ");
    Serial.print(deriv);
    Serial.print(" T: ");
    Serial.println(turn);*/
  }
}

boolean VisionBase::sensorDetection()
{
  return frontLeft.detect() || frontRight.detect();
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
    carpetClaw.attach(carpetClawPin);
    carpetClaw.write(60);
  }
}

void VisionBase::moveBeacon(int value)
{
  if(!beaconServo.attached())
    beaconServo.attach(beaconServoPin);
  if(value != lastBeaconServoValue)
  {
    beaconServo.write(value);
    lastBeaconServoValue = value;
  }
}

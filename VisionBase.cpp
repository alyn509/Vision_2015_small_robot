#include "VisionBase.h"

void VisionBase::init()
{
  frontLeft.initPin(frontLeftSensorPin);
  frontRight.initPin(frontRightSensorPin);
  
  leftMotor.init(leftMotorFw);
  rightMotor.init(rightMotorFw);
  
  leftEncoder.init(leftEncoderStepPin);
  rightEncoder.init(rightEncoderStepPin);
       
/*  carpetClaw.attach(servoPin2);
  carpetClaw.write(0);
  carpetClaw.detach();
  beaconServo.attach(servoPin1);
  beaconServo.write(40);
  beaconServo.detach();*/
}

void VisionBase::moveForward(unsigned char pwmLeft, unsigned char pwmRight)
{  
  if(!newMovement || isResuming)
  {  
    directionMovement = FRONT;
    leftMotor.moveForward(pwmLeft);
    rightMotor.moveForward(pwmRight);
    if(!isResuming)
    {
      leftEncoder.resetPosition();
      rightEncoder.resetPosition();
    }
    isResuming = false;
  }
}
void VisionBase::turnLeft(unsigned char pwmLeft, unsigned char pwmRight)
{  
  if(!newMovement || isResuming)
  {  
    directionMovement = LEFT;
    rightMotor.moveForward(pwmRight);
    if(!isResuming)
    {
      leftEncoder.resetPosition();
      rightEncoder.resetPosition();
    }
    isResuming = false;
  }
}

void VisionBase::turnRight(unsigned char pwmLeft, unsigned char pwmRight)
{    
  if(!newMovement || isResuming)
  {  
    directionMovement = RIGHT;
    leftMotor.moveForward(pwmLeft);
    if(!isResuming)
    {
      leftEncoder.resetPosition();
      rightEncoder.resetPosition();
    }
    isResuming = false;
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
    case 0:    
      moveForward(20,20);
      doDistanceInCM(80,1);
      break;
    case 1:    
      turnRight(80,80);
      doAngleRotation(90,2);
      break;
    case 2:   
      moveForward(20,20);
      doDistanceInCM(70,3);
      break;
    case 3:   
      moveForward(20,20);
      releaseCarpets();
      doDistanceInCM(10,4);
      break;
    case 4:    
      stopNow();
      break;
    case PAUSED:
      break;
    default:
      state.doLoop();
  }
  leftEncoder.updatePosition();
  rightEncoder.updatePosition();
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
    int difference = leftEncoder.getPosition() - rightEncoder.getPosition();
    int deriv = difference - last;
    last = difference;
    integral += difference;
    int turn = 2.0 * difference + 0.0 * integral + 0.0 * deriv;
    if (turn > 70) turn = 70;
    if (turn < -70) turn = -70;
    if (leftMotor.isOn)
      leftMotor.moveForward(80 - turn);
    if (rightMotor.isOn)
      rightMotor.moveForward(80 + turn);
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
    carpetClaw.attach(servoPin2);
    carpetClaw.write(60);
  }
}

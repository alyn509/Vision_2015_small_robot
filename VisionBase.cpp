#include "VisionBase.h"

void VisionBase::init()
{
  frontLeft.initPin(frontLeftSensorPin);
  frontRight.initPin(frontRightSensorPin);
  
  rightMotor.init(rightMotorFw, rightMotorBw);
  leftMotor.init(leftMotorFw, leftMotorBw);

  leftEncoder.init(leftEncoderStepPin);
  rightEncoder.init(rightEncoderStepPin);
  
  servo1.attach(servoPin1);
  servo1.write(0);
  servo2.attach(servoPin2);
  servo2.write(0);
}

void VisionBase::moveForward(unsigned char pwmLeft, unsigned char pwmRight)
{  
  if(!newMovement)
  {  
    directionMovement = FRONT;
    leftMotor.moveForward(pwmLeft);
    rightMotor.moveForward(pwmRight);
  }
}

void VisionBase::moveBackward(unsigned char pwmLeft, unsigned char pwmRight)
{    
  if(!newMovement)
  {  
    directionMovement = BACK;
    leftMotor.moveBackward(pwmLeft);
    rightMotor.moveBackward(pwmRight);  
  }  
}

void VisionBase::turnLeft(unsigned char pwmLeft, unsigned char pwmRight)
{  
  if(!newMovement)
  {  
    directionMovement = LEFT;
    leftMotor.moveBackward(pwmLeft);
    rightMotor.moveForward(pwmRight);
  }
}

void VisionBase::turnRight(unsigned char pwmLeft, unsigned char pwmRight)
{    
  if(!newMovement)
  {  
    directionMovement = RIGHT;
    leftMotor.moveForward(pwmLeft);
    rightMotor.moveBackward(pwmRight);
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
float VisionBase::cmToSteps(int value)
{
  return (value * encoderResolution) / (PI * wheelDiameter);
}
float VisionBase::angleToSteps(int value)
{
  return (value * distanceBetweenWheels * PI) / 360.0;
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
    if(!leftMotor.isOn && !rightMotor.isOn)
    {
      newMovement = false;
      state = nextState;
    }
  }
}
void VisionBase::doAngleRotation(int dist, int nextState)
{
  float steps = angleToSteps(dist);
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
    if(!leftMotor.isOn && !rightMotor.isOn)
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
}

void VisionBase::doLoop()
{
  
  switch (state)
  {
    case 0:    
      moveForward(20,20);
      doDistanceInCM(1500,1);
      //state.wait(3000, 0);
      break;
    case 1:    
      stopNow();
      state.wait(500, 1);
      break;
    default:
      state.doLoop();
  }
  leftEncoder.updatePosition(leftMotorDir());
  rightEncoder.updatePosition(rightMotorDir());
}
int integral, last;
void VisionBase::update()
{
  /*if (rightEncoder.getPosition() == 100)
  {
    leftEncoder.currentPosition = 0;
    leftMotor.stopMotor();
  }
  if (rightEncoder.getPosition() == 100)
  {
    rightEncoder.currentPosition = 0;
    rightMotor.stopMotor();
  }*/
  Serial.print(" L: ");
  Serial.print(leftEncoder.getPosition());
  Serial.print(" R: ");
  Serial.print(rightEncoder.getPosition());

  //if()
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

    Serial.print(" E: ");
    Serial.print(difference);
    Serial.print(" I: ");
    Serial.print(integral);
    Serial.print(" D: ");
    Serial.print(deriv);
    Serial.print(" T: ");
    Serial.println(turn);
  }
}

boolean VisionBase::frontDetected()
{
  return frontLeft.detect() || frontRight.detect();
}


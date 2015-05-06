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
    if(!leftMotor.isOn && !rightMotor.isOn)
    {
      newMovement = false;
      state = nextState;
    }
  }
}
void VisionBase::doAngleRotation(int dist, int nextState)
{
  float steps = cmToSteps(angleToSteps(dist));
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
      doDistanceInCM(80,1);
      break;
    case 1:    
      turnRight(20,20);
      doAngleRotation(90,2);
      break;
    case 2:    
      moveForward(20,20);
      doDistanceInCM(80,3);
      break;
    case 3:    
      stopNow();
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
  Serial.print(" L: ");
  Serial.print(leftEncoder.getPosition());
  Serial.print(" R: ");
  Serial.print(rightEncoder.getPosition());

  int difference = leftEncoder.getPosition() - rightEncoder.getPosition();
  int deriv = difference - last;
  last = difference;
  integral += difference;
  int turn = 2.0 * difference + 0.0 * integral + 0.0 * deriv;
  if (turn > 70) turn = 70;
  if (turn < -70) turn = -70;
  if(directionMovement == FRONT)
  {
    if (leftMotor.isOn)
      leftMotor.moveForward(80 - turn);
    if (rightMotor.isOn)
      rightMotor.moveForward(80 + turn);
  }
  else if(directionMovement == RIGHT)
  {      
    if (leftMotor.isOn)
      leftMotor.moveForward(80 - turn);
    if (rightMotor.isOn)
      rightMotor.moveBackward(80 + turn);
  }
  else if(directionMovement == LEFT)
  {      
    if (leftMotor.isOn)
      leftMotor.moveBackward(80 - turn);
    if (rightMotor.isOn)
      rightMotor.moveForward(80 + turn);
  }
  else if(directionMovement == BACK)
  {      
    if (leftMotor.isOn)
      leftMotor.moveBackward(80 - turn);
    if (rightMotor.isOn)
      rightMotor.moveBackward(80 + turn);
  }

  Serial.print(" E: ");
  Serial.print(difference);
  Serial.print(" I: ");
  Serial.print(integral);
  Serial.print(" D: ");
  Serial.print(deriv);
  Serial.print(" T: ");
  Serial.println(turn);
}

boolean VisionBase::frontDetected()
{
  return frontLeft.detect() || frontRight.detect();
}


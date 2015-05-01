#include "VisionBase.h"

void VisionBase::init()
{
  frontLeft.initPin(frontLeftSensorPin);
  frontMid.initPin(frontMidSensorPin);
  frontRight.initPin(frontRightSensorPin);
  
  rightMotor.init(rightMotorFw, rightMotorBw);
  leftMotor.init(leftMotorFw, leftMotorBw);

  leftEncoder.init(leftEncoderStepPin);
  rightEncoder.init(rightEncoderStepPin);
  
  servo1.attach(servoPin1);
  servo1.write(0);
  servo2.attach(servoPin2);
  servo2.write(0);
  servo3.attach(servoPin3);
  servo3.write(0);
}

void VisionBase::moveForward(unsigned char pwmLeft, unsigned char pwmRight)
{       
  directionMovement = FRONT;
  leftMotor.moveForward(pwmLeft);
  rightMotor.moveForward(pwmRight);
}

void VisionBase::moveBackward(unsigned char pwmLeft, unsigned char pwmRight)
{    
  directionMovement = BACK;
  leftMotor.moveBackward(pwmLeft);
  rightMotor.moveBackward(pwmRight);      
}

void VisionBase::turnLeft(unsigned char pwmLeft, unsigned char pwmRight)
{
  directionMovement = LEFT;
  leftMotor.moveBackward(pwmLeft);
  rightMotor.moveForward(pwmRight);
}

void VisionBase::turnRight(unsigned char pwmLeft, unsigned char pwmRight)
{  
  directionMovement = RIGHT;
  leftMotor.moveForward(pwmLeft);
  rightMotor.moveBackward(pwmRight);
}
bool VisionBase::leftMotorDir()
{
  return (directionMovement / 2) % 2;
}

bool VisionBase::rightMotorDir()
{  
  return directionMovement % 2;
}

void VisionBase::stopNow()
{  
  directionMovement = NONE;        
  rightMotor.stopMotor();      
  leftMotor.stopMotor();
}
void VisionBase::doLoop()
{
  
  switch (state)
  {
    case 0:    
      moveForward(40,40);
      state.wait(4000, STATE_NEXT);
      break;
    case 1:    
      stopNow();
      state.wait(1000, 0);
      break;
    default:
      state.doLoop();
  }
}
void VisionBase::update()
{
  leftEncoder.updatePosition(leftMotorDir());
  rightEncoder.updatePosition(rightMotorDir());
  if(directionMovement == FRONT)
  {
    int difference = leftEncoder.getPosition() - rightEncoder.getPosition();
    if (difference >= 4)
    {
      leftMotor.moveForward(10);
      rightMotor.moveForward(40);
    }
    else if(difference <= -4)
    {
      leftMotor.moveForward(40);
      rightMotor.moveForward(10);
    }
    else
    {
      leftMotor.moveForward(40);
      rightMotor.moveForward(40);
    }
  }
}

boolean VisionBase::frontDetected()
{
  return frontLeft.detect() || frontRight.detect();
}


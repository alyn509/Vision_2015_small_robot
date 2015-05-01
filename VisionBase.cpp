#include "VisionBase.h"

void VisionBase::init()
{
  frontLeft.initPin(frontLeftSensorPin);
  frontMid.initPin(frontMidSensorPin);
  frontRight.initPin(frontRightSensorPin);
  
  rightMotor.init(rightMotorFw, rightMotorBw);
  leftMotor.init(leftMotorFw, leftMotorBw);

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
  moveForward(40,40);
  //delay(1000);
  //stopNow();
  delay(1000);
  moveBackward(40,40);
  //delay(1000);
  //stopNow();
  delay(1000);
}

boolean VisionBase::frontDetected()
{
  return frontLeft.detect() || frontRight.detect();
}


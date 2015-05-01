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

void VisionBase::moveForward()
{       
  directionMovement = FRONT;       
  rightMotor.moveForward();      
  leftMotor.moveForward();
}

void VisionBase::moveBackward()
{    
  directionMovement = BACK;   
  rightMotor.moveBackward();      
  leftMotor.moveBackward();
}

void VisionBase::turnLeft()
{
  directionMovement = LEFT;     
  rightMotor.moveForward();      
  leftMotor.moveBackward();
}

void VisionBase::turnRight()
{  
  directionMovement = RIGHT;        
  rightMotor.moveBackward();      
  leftMotor.moveForward();
}

void VisionBase::stopNow()
{  
  directionMovement = NONE;        
  rightMotor.stopMotor();      
  leftMotor.stopMotor();
}
void VisionBase::doLoop()
{  
  int servVal = 0;
  int fwbw = 0;
  while(1)
  {
     moveForward();
    servo3.write(servVal);
    servo3.write(servVal);
    servo3.write(servVal);
     if(fwbw == 0)
       servVal +=5;
     else
       servVal -=5;
     if(servVal >=90)
       fwbw = 1;
     if(servVal <5)
       fwbw = 0;
     delay(300);
     moveBackward();
    servo3.write(servVal);
    servo3.write(servVal);
    servo3.write(servVal);
     if(fwbw == 0)
       servVal +=5;
     else
       servVal -=5;
     if(servVal >=90)
       fwbw = 1;
     if(servVal <5)
       fwbw = 0;
     delay(300);
  }
}

boolean VisionBase::frontDetected()
{
  return frontLeft.detect() || frontRight.detect();
}


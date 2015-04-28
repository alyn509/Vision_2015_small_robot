#include "VisionBase.h"

void VisionBase::init()
{
  frontLeft.initPin(frontLeftSensorPin);
  //frontFront.initPin(frontFrontSensorPin);
  frontRight.initPin(frontRightSensorPin);
  
  backLeft.initPin(backLeftSensorPin);
  backRight.initPin(backRightSensorPin);
  backLow.initPin(backLowSensorPin);

  rightMotor.init(rightMotorFw, rightMotorBw);
  leftMotor.init(leftMotorFw, leftMotorBw);
  
  sensorScanner.attach(sensorScannerPin);
  sensorScanner.write(sensorScannerMiddle);
  
  directionMovement = NONE;
  obstructionDetected = false;
  ignoredSensors = false;
  pinMode(colorRedPin, INPUT);
  oppositeSide = (digitalRead(colorRedPin) == HIGH);// = false;
}

void VisionBase::moveForward(float distance, unsigned long step_delay)
{       
  directionMovement = FRONT;       
  rightMotor.moveForward();      
  leftMotor.moveForward();
}

void VisionBase::moveBackward(float distance, unsigned long step_delay)
{    
  directionMovement = BACK;   
  rightMotor.moveBackward();      
  leftMotor.moveBackward();
}

void VisionBase::turnLeft(int angle)
{
  directionMovement = LEFT;     
  rightMotor.moveForward();      
  leftMotor.moveBackward();
}

void VisionBase::turnRight(int angle)
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

boolean VisionBase::frontDetected()
{
  return frontLeft.detect() || frontRight.detect();
}

boolean VisionBase::backDetected()
{
  return backLeft.detect() || backRight.detect() || backLow.detect();
}

void VisionBase::checkObstructions()
{
  obstructionDetected = false;
  if (frontDetected() && !ignoredSensors && directionMovement == FRONT)
  {
    obstructionDetected = true;
  }
  if (backDetected() && !ignoredSensors && directionMovement == BACK)
    obstructionDetected = true;
}

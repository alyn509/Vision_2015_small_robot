#include "VisionBase.h"
#include <HMC5883L.h>

HMC5883L compass;

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
      moveForward(400,400);
      state.wait(3000, 0);
      break;
    case 1:    
      stopNow();
      state.wait(2000, 0);
      break;
    default:
      state.doLoop();
  }
}

float integral, last;
void VisionBase::update()
{
  leftEncoder.updatePosition(leftMotorDir());
  rightEncoder.updatePosition(rightMotorDir());
  
  //if(directionMovement == FRONT)
  if (0)
  {
    float difference = leftEncoder.getPosition() - rightEncoder.getPosition() * 1.09;
    float deriv = difference - last;
    last = difference;
    integral += difference;
    int turn = 1.5 * difference + 0.1 * integral + 0.0 * deriv;
    if (turn > 100) turn = 100;
    if (turn < -100) turn = -100;
    leftMotor.moveForward(400 + turn);
    rightMotor.moveForward(400 - turn);

    Serial.print("L: ");
    Serial.print(leftEncoder.getPosition());
    Serial.print(" R: ");
    Serial.print(rightEncoder.getPosition());
    Serial.print(" dif: ");
    Serial.print(difference);
    Serial.print(" int: ");
    Serial.print(integral);
    Serial.print(" deriv: ");
    Serial.print(deriv);
    Serial.print(" turn: ");
    Serial.println(turn);
  }
}

boolean VisionBase::frontDetected()
{
  return frontLeft.detect() || frontRight.detect();
}


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
      moveForward(55,60);
      state.wait(3000, 0);
      break;
    case 1:    
      stopNow();
      state.wait(500, 0);
      break;
    default:
      state.doLoop();
  }
}
int integral, last;
void VisionBase::update()
{
  leftEncoder.updatePosition(leftMotorDir());
  rightEncoder.updatePosition(rightMotorDir());
  if (rightEncoder.getPosition() == 100)
  {
    leftEncoder.currentPosition = 0;
    leftMotor.stopMotor();
  }
  if (rightEncoder.getPosition() == 100)
  {
    rightEncoder.currentPosition = 0;
    rightMotor.stopMotor();
  }
  /*if(directionMovement == FRONT)
  {
    int difference = leftEncoder.getPosition() - rightEncoder.getPosition();
    int deriv = difference - last;
    last = difference;
    integral += difference;
    int turn = 0.5 * difference + 0.1 * integral + 25.0 * deriv;
    if (turn > 30) turn = 30;
    if (turn < -30) turn = -30;
    leftMotor.moveForward(80 + turn);
    rightMotor.moveForward(80 - turn);
    Serial.print("dif: ");
    Serial.print(difference);
    Serial.print(" int: ");
    Serial.print(integral);
    Serial.print(" deriv: ");
    Serial.print(deriv);
    Serial.print(" turn: ");
    Serial.println(turn);
  }*/
}

boolean VisionBase::frontDetected()
{
  return frontLeft.detect() || frontRight.detect();
}


#ifndef VisionBase_h
#define VisionBase_h

#include "Arduino.h"
#include "VisionDC.h"
#include "VisionSensor.h"
//#include "VisionEncoders.h"
#include "pins.h"
//#include "constants.h"
//#include <elapsedMillis.h>
#include <SoftwareServo.h>

#define NONE 4
#define FRONT 3
#define BACK 0
#define LEFT 1
#define RIGHT 2

class VisionBase {
  public:
    void init();
    
    void moveForward(unsigned char pwmLeft, unsigned char pwmRight);
    void moveBackward(unsigned char pwmLeft, unsigned char pwmRight);
    
    boolean frontDetected();
    
    void turnLeft(unsigned char pwmLeft, unsigned char pwmRight);
    void turnRight(unsigned char pwmLeft, unsigned char pwmRight);
    
    bool leftMotorDir();
    bool rightMotorDir();
    
    void doLoop();
    
    void stopNow();    
  public:
    SoftwareServo servo1, servo2, servo3;
    
    VisionDC leftMotor, rightMotor;
    VisionSensor frontLeft, frontMid, frontRight;
    
    int directionMovement = NONE;
};

#endif


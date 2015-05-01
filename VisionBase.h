#ifndef VisionBase_h
#define VisionBase_h

#include "Arduino.h"
#include "VisionDC.h"
#include "VisionSensor.h"
#include "VisionEncoders.h"
#include "pins.h"
#include "constants.h"
#include <elapsedMillis.h>
#include <Servo.h>

#define NONE 4
#define FRONT 3
#define BACK 0
#define LEFT 1
#define RIGHT 2

class VisionBase {
  public:
    void init();
    
    void moveForward();
    void moveBackward();
    
    boolean frontDetected();
    
    void turnLeft();
    void turnRight();
    
    bool leftMotorDir();
    bool rightMotorDir();
    
    void doLoop();
    
    void stopNow();    
  public:
    Servo servo1;
    Servo servo2;
    Servo servo3;
    
    VisionDC leftMotor, rightMotor;
    VisionSensor frontLeft, frontMid, frontRight;
    
    int directionMovement = NONE;
};

#endif


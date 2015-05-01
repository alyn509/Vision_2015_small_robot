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

#define NONE 0
#define FRONT 1
#define BACK 2
#define LEFT 3
#define RIGHT 4

class VisionBase {
  public:
    void init();
    
    void moveForward();
    void moveBackward();
    
    boolean frontDetected();
    
    void turnLeft();
    void turnRight();
    
    void doLoop();
    
    void stopNow();
    
  public:
    Servo servo1;
    Servo servo2;
    Servo servo3;
    
    VisionDC leftMotor, rightMotor;
    VisionSensor frontLeft, frontMid, frontRight;
    
    int directionMovement;
};

#endif


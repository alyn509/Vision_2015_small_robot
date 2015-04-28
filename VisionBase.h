#ifndef VisionBase_h
#define VisionBase_h

#include "Arduino.h"
#include "VisionDC.h"
#include "VisionSensor.h"
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
    Servo sensorScanner;
    elapsedMillis sensorToggleTimer;
    void moveForward(float distance, unsigned long step_delay);
    void moveBackward(float distance, unsigned long step_delay);
    boolean frontDetected();
    //boolean leftDetected();
    //boolean rightDetected();
    boolean backDetected();
    void checkObstructions();
    void turnLeft(int angle);
    void turnRight(int angle);
    void stopNow();
  public:
    VisionDC leftMotor, rightMotor;
    VisionSensor backLeft, backRight, backLow, frontLeft, frontFront, frontRight;
    int directionMovement;
    boolean oppositeSide;
    boolean ignoredSensors;
    boolean obstructionDetected;
};

#endif


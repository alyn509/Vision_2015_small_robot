#ifndef VisionBase_h
#define VisionBase_h

#include "Arduino.h"
#include "VisionDC.h"
#include "VisionSensor.h"
#include "VisionState.h"
#include "VisionEncoders.h"
#include "pins.h"
#include "constants.h"
#include <Servo.h>

#define NONE 4
#define FRONT 3
#define BACK 0
#define LEFT 1
#define RIGHT 2

#define PAUSED 500

class VisionBase {
  public:
    void init();
    
    void moveForward(unsigned char pwmLeft, unsigned char pwmRight);
    
    boolean sensorDetection();
    
    void turnLeft(unsigned char pwmLeft, unsigned char pwmRight);
    void turnRight(unsigned char pwmLeft, unsigned char pwmRight);
    
    void doDistanceInCM(int dist, int nextState);
    void doAngleRotation(int dist, int nextState);
    
    bool leftMotorDir();
    bool rightMotorDir();    
    
    float cmToSteps(float value);
    float angleToSteps(float value);

    void doLoop();
    void update();
    
    void pause();
    void unpause();
    
    void releaseCarpets();
        
    void stopNow();    
  public:
    Servo beaconServo, carpetClaw;
    
    VisionEncoders leftEncoder, rightEncoder;
    VisionState state;
    
    VisionDC leftMotor, rightMotor;
    VisionSensor frontLeft, frontRight;
    
    int directionMovement = NONE;
    int stateBeforePause;
    float lastPositionLeft = 0;
    float lastPositionRight = 0;
    
    bool isPaused = false;
    bool isResuming = false;
    bool isStopped = false;
    bool ignoredSensors = false;
    bool newMovement = false;
};

#endif


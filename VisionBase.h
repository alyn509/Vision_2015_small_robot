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
#include <elapsedMillis.h>

#define NONE 4
#define FRONT 3
#define BACK 0
#define LEFT 1
#define RIGHT 2

#define PAUSED 500
#define OVER 501
#define HOMOLOGATION 400
#define YELLOW_SIDE 0
#define GREEN_SIDE 100

#define CLIMB_STAIRS 50
class VisionBase {
  public:
    void init();
    
    void moveForward(int pwmv);
    
    boolean sensorDetection();
    
    void turnLeft(int pwmv);
    void turnRight(int pwmv);
    
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
    
    void lowerBeacon();
    void resumeBeacon();
    
    void releaseCarpets();
    void moveBeacon(int value);
        
    void stopNow();    
  public:
    Servo beaconServo, carpetClaw;
    
    VisionEncoders leftEncoder, rightEncoder;
    VisionState state;
    
    VisionDC leftMotor, rightMotor;
    VisionSensor frontLeft, frontRight;
    VisionSensor color;
    
    int pwmValue = 0;
    int directionMovement = NONE;
    int stateBeforePause;
    float lastPositionLeft = 0;
    float lastPositionRight = 0;
    int lastBeaconServoValue = 0;
    
    bool isPaused = false;
    bool isResuming = false;
    bool isStopped = false;
    bool ignoredSensors = false;
    bool newMovement = false;
};

#endif


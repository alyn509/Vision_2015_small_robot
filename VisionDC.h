  #ifndef VisionDC_h
  #define VisionDC_h
  
  #include "Arduino.h"
//  #include <elapsedMillis.h>
//  #include "VisionState.h"
  
  
  class VisionDC {
    public:
      int forwardPin, backwardPin;
    public:
      void init(int fwPin, int bwPin);
      void moveForward(unsigned char pwm);
      void moveBackward(unsigned char pwm);
      void stopMotor();
  };
  
  #endif


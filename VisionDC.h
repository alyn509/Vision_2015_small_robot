  #ifndef VisionDC_h
  #define VisionDC_h
  
  #include "Arduino.h"
  #include <elapsedMillis.h>
  #include "VisionState.h"
  
  
  class VisionDC {
    public:
      int forwardPin, backwardPin;
    public:
      void init(int fwPin, int bwPin);
      void moveForward();
      void moveBackward();
      void stopMotor();
  };
  
  #endif


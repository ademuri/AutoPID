#pragma once

#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>

#else
// Declare Arduino functions, so that this library can be used in native tests.
extern uint32_t millis();
extern float constrain(float, float, float);

#endif

class AutoPID {
 public:
  // Constructor - takes pointer inputs for control variales, so they are
  // updated automatically
  AutoPID(float outputMin, float outputMax, float Kp, float Ki, float Kd);
  // Allows manual adjustment of gains
  void setGains(float Kp, float Ki, float Kd);
  // Sets bang-bang control ranges, separate upper and lower offsets, zero for
  // off
  void setBangBang(float bangOn, float bangOff);
  // Sets bang-bang control range +-single offset
  void setBangBang(float bangRange);
  // Allows manual readjustment of output range
  void setOutputRange(float outputMin, float outputMax);
  // Allows manual adjustment of time step (default 1000ms)
  void setTimeStep(unsigned long timeStep);

  void setSetPoint(float setpoint);

  // Returns true when at set point (+-threshold)
  bool atSetPoint(float threshold);
  // Runs PID calculations when needed. Should be called repeatedly in loop.
  // Automatically reads input and sets output via pointers
  virtual void run(float input);
  // Stops PID functionality, output sets to
  void stop();
  virtual void reset();
  bool isStopped();

  float getIntegral();
  void setIntegral(float integral);

  float getOutput();

 private:
  float _Kp;
  float _Ki;
  float _Kd;
  float _integral = 0;
  float _previousError = 0;
  float _bangOn = 0;
  float _bangOff = 0;
  float _input = 0;
  float _setpoint = 0;
  float _output = 0;
  float _outputMin = 0;
  float _outputMax = 0;
  unsigned long _timeStep = 0;
  unsigned long _lastStep = 0;
  bool _stopped = false;

};  // class AutoPID

class AutoPIDRelay : public AutoPID {
 public:
  AutoPIDRelay(unsigned long pulseWidth, float Kp, float Ki, float Kd)
      : AutoPID(/*outputMin=*/0, /*outputMax=*/1.0, Kp, Ki, Kd),
        _pulseWidth(pulseWidth){};

  void run(float input) override;

  bool getRelayState();

  void reset() override;

 private:
  bool _relayState = false;
  const unsigned long _pulseWidth;
  unsigned long _pulseOffset = 0;
  float _pulseValue = 0;
  bool _hasRun = false;
};  // class AutoPIDRelay

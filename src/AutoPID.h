#pragma once

#ifdef ARDUINO
#include <Arduino.h>

#else
// Declare Arduino functions, so that this library can be used in native tests.
extern unsigned long millis();
extern float abs(float);
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
  float _Kp, _Ki, _Kd;
  float _integral, _previousError;
  float _bangOn, _bangOff;
  float _input;
  float _setpoint;
  float _output;
  float _outputMin, _outputMax;
  unsigned long _timeStep, _lastStep;
  bool _stopped;

};  // class AutoPID

class AutoPIDRelay : public AutoPID {
 public:
  AutoPIDRelay(unsigned long pulseWidth, float Kp, float Ki, float Kd)
      : AutoPID(/*outputMin=*/0, /*outputMax=*/1.0, Kp, Ki, Kd), _pulseWidth(pulseWidth) {};

  void run(float input) override;

  bool getRelayState();

  void reset() override;

 private:
  bool _relayState;
  const unsigned long _pulseWidth;
  unsigned long _pulseOffset;
  float _pulseValue;
  bool _hasRun = false;
};  // class AutoPIDRelay

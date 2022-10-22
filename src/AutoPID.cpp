#include "AutoPID.h"

AutoPID::AutoPID(float outputMin, float outputMax, float Kp, float Ki,
                 float Kd) {
  _outputMin = outputMin;
  _outputMax = outputMax;
  setGains(Kp, Ki, Kd);
  _timeStep = 1000;
}  // AutoPID::AutoPID

void AutoPID::setGains(float Kp, float Ki, float Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}  // AutoPID::setControllerParams

void AutoPID::setBangBang(float bangOn, float bangOff) {
  _bangOn = bangOn;
  _bangOff = bangOff;
}  // void AutoPID::setBangBang

void AutoPID::setBangBang(float bangRange) {
  setBangBang(bangRange, bangRange);
}  // void AutoPID::setBangBang

void AutoPID::setOutputRange(float outputMin, float outputMax) {
  _outputMin = outputMin;
  _outputMax = outputMax;
}  // void AutoPID::setOutputRange

void AutoPID::setTimeStep(unsigned long timeStep) { _timeStep = timeStep; }

void AutoPID::setSetPoint(float setpoint) { _setpoint = setpoint; }

bool AutoPID::atSetPoint(float threshold) {
  // Note: avoid using abs because it's an Arduino macro, and std::abs is not available on older platforms (e.g. Uno)
  float diff = _setpoint - _input;
  if (diff < 0) {
    diff = -diff;
  }
  return diff <= threshold;
}  // bool AutoPID::atSetPoint

void AutoPID::run(float input) {
  _input = input;
  if (_stopped) {
    _stopped = false;
    reset();
  }
  // if bang thresholds are defined and we're outside of them, use bang-bang
  // control
  if (_bangOn && ((_setpoint - _input) > _bangOn)) {
    _output = _outputMax;
    _lastStep = millis();
  } else if (_bangOff && ((_input - _setpoint) > _bangOff)) {
    _output = _outputMin;
    _lastStep = millis();
  } else {  // otherwise use PID control
    unsigned long _dT =
        millis() - _lastStep;  // calculate time since last update
    if (_dT >= _timeStep) {    // if long enough, do PID calculations
      _lastStep = millis();
      float _error = _setpoint - _input;
      _integral +=
          (_error + _previousError) / 2 * _dT / 1000.0;  // Riemann sum integral
      //_integral = constrain(_integral, _outputMin/_Ki, _outputMax/_Ki);
      float _dError = (_error - _previousError) / (_dT / 1000.0);  // derivative
      _previousError = _error;
      float PID = (_Kp * _error) + (_Ki * _integral) + (_Kd * _dError);
      //*_output = _outputMin + (constrain(PID, 0, 1) * (_outputMax -
      //_outputMin));
      _output = constrain(PID, _outputMin, _outputMax);
    }
  }
}  // void AutoPID::run

void AutoPID::stop() {
  _stopped = true;
  reset();
}
void AutoPID::reset() {
  _lastStep = millis();
  _integral = 0;
  _previousError = 0;
}

bool AutoPID::isStopped() { return _stopped; }

float AutoPID::getIntegral() { return _integral; }

void AutoPID::setIntegral(float integral) { _integral = integral; }

float AutoPID::getOutput() { return isStopped() ? 0 : _output; }

void AutoPIDRelay::run(float input) {
  AutoPID::run(input);
  if (!_hasRun) {
    _pulseOffset = millis();
    _hasRun = true;
  }
  _relayState = ((millis() - _pulseOffset) % _pulseWidth) < _pulseValue;
}

bool AutoPIDRelay::getRelayState() { return _relayState; }

void AutoPIDRelay::reset() {
  AutoPID::reset();
  _pulseOffset = millis();
}

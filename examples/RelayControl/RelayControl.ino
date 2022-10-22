#include <Arduino.h>
#include <AutoPID.h>

constexpr int kInput = 0;
constexpr int kSetpointInput = 1;
constexpr int kOutput = 2;
constexpr int kLed = 13;

constexpr unsigned long pulseWidth = 1000;

AutoPIDRelay pid{pulseWidth, /*Kp=*/0.1, /*Ki=*/0.1, /*Kd=*/0.5};

void setup() {
  pinMode(kInput, INPUT);
  pinMode(kSetpointInput, INPUT);
  pinMode(kOutput, OUTPUT);
  pinMode(kLed, OUTPUT);

  // Run at most every 10ms
  pid.setTimeStep(10);
}

void loop() {
  pid.setSetPoint(analogRead(kSetpointInput));
  pid.run(analogRead(kInput));
  digitalWrite(kOutput, pid.getRelayState());
  digitalWrite(kLed, pid.getRelayState());
}
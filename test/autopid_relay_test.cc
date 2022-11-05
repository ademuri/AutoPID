#include <gtest/gtest.h>

#include "AutoPID.h"

extern uint32_t millis_;

namespace {

TEST(AutoPIDRelay, RelayOutput) {
  AutoPIDRelay pid(/*pulseWidth=*/10, /*Kp=*/0.1, /*Ki=*/0, /*Kd=*/0);
  pid.setTimeStep(10);
  pid.setSetPoint(0);
  for (uint32_t t = 0; t < 20; t++) {
    pid.run(0);
    EXPECT_FALSE(pid.getRelayState());
    millis_++;
  }

  pid.setSetPoint(1);
  pid.run(0);
  EXPECT_TRUE(pid.getRelayState()) << millis();
  millis_++;
  for (uint32_t t = 0; t < 9; t++) {
    pid.run(0);
    EXPECT_FALSE(pid.getRelayState()) << millis();
    millis_++;
  }

  pid.setGains(/*Kp=*/0.2, /*Ki=*/0, /*Kd=*/0);
  pid.run(0);
  EXPECT_TRUE(pid.getRelayState()) << millis();
  millis_++;
  pid.run(0);
  EXPECT_TRUE(pid.getRelayState()) << millis();
  millis_++;
  for (uint32_t t = 0; t < 8; t++) {
    pid.run(0);
    EXPECT_FALSE(pid.getRelayState()) << millis();
    millis_++;
  }

  pid.setGains(/*Kp=*/1, /*Ki=*/0, /*Kd=*/0);
  for (uint32_t t = 0; t < 10; t++) {
    pid.run(0);
    EXPECT_TRUE(pid.getRelayState()) << millis();
    millis_++;
  }
}

};  // namespace

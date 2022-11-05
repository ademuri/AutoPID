#include "AutoPID.h"

#include <gtest/gtest.h>

uint32_t millis_ = 0;
uint32_t millis() { return millis_; }

float constrain(float input, float min, float max) {
  input = std::min(input, max);
  input = std::max(input, min);
  return input;
}

namespace {

TEST(AutoPID, OnlyP) {
  AutoPID pid{/*outputMin=*/0, /*outputMax=*/1, /*Kp=*/0.1, /*Ki=*/0, /*Kd=*/0};
  pid.setTimeStep(1);
  pid.setSetPoint(0);
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);
  EXPECT_TRUE(pid.atSetPoint(0.01));
  EXPECT_FALSE(pid.isStopped());

  millis_++;
  pid.setSetPoint(1);
  pid.run(0);
  EXPECT_FALSE(pid.atSetPoint(0.01));
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.1);

  millis_++;
  pid.run(0.1);
  EXPECT_FALSE(pid.atSetPoint(0.01));
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.09);

  millis_++;
  pid.run(0.9);
  EXPECT_FALSE(pid.atSetPoint(0.01));
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.01);

  millis_++;
  pid.run(0.95);
  EXPECT_FALSE(pid.atSetPoint(0.01));
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.005);

  millis_++;
  pid.run(1);
  EXPECT_TRUE(pid.atSetPoint(0.01));
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  millis_++;
  pid.run(2);
  EXPECT_FALSE(pid.atSetPoint(0.01));
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);
}

TEST(AutoPID, OnlyI) {
  AutoPID pid{/*outputMin=*/0, /*outputMax=*/1, /*Kp=*/0, /*Ki=*/0.1, /*Kd=*/0};
  pid.setTimeStep(1);
  pid.setSetPoint(0);
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  millis_ += 1000;
  pid.setSetPoint(1);
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.05);

  millis_ += 1000;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.15);

  millis_ += 1000;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.25);

  millis_ += 1000;
  pid.run(1);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.3);

  millis_ += 1000;
  pid.run(1);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.3);

  millis_ += 1000;
  pid.run(2);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.25);

  millis_ += 1000;
  pid.run(2);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.15);
}

TEST(AutoPID, OnlyD) {
  AutoPID pid{/*outputMin=*/0, /*outputMax=*/1, /*Kp=*/0, /*Ki=*/0, /*Kd=*/0.1};
  pid.setTimeStep(1);
  pid.setSetPoint(0);
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  millis_ += 1000;
  pid.setSetPoint(1);
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.1);

  millis_ += 1000;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  millis_ += 1000;
  pid.run(1);
  // Should be clamped to 0
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  pid.setGains(/*Kp=*/0.1, /*Ki=*/0, /*Kd=*/0.1);
  millis_ += 1000;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.2);
  millis_ += 1000;
  pid.run(0.5);
  // P would set this to 0.05
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);
}

TEST(AutoPID, LimitsIntegratorWindup) {
  AutoPID pid{/*outputMin=*/0, /*outputMax=*/1, /*Kp=*/0, /*Ki=*/1, /*Kd=*/0};
  pid.setTimeStep(1);
  pid.setSetPoint(0);
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);
  EXPECT_TRUE(pid.atSetPoint(0.01));

  millis_ += 1000;
  pid.setSetPoint(1);
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.5);

  millis_ += 1000;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 1);

  millis_ += 1000;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 1);

  pid.setSetPoint(0);
  millis_ += 1000;
  pid.run(1);
  // Since we're approximating the integral, it takes a cycle to catch up
  EXPECT_FLOAT_EQ(pid.getOutput(), 1);

  millis_ += 1000;
  pid.run(1);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  millis_ += 1000;
  pid.run(1);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  millis_ += 1000;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  millis_ += 1000;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);
}

TEST(AutoPID, RespectsTimeStep) {
  AutoPID pid{/*outputMin=*/0, /*outputMax=*/1, /*Kp=*/0.1, /*Ki=*/0, /*Kd=*/0};
  pid.setTimeStep(10);
  pid.setSetPoint(0);
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  pid.setSetPoint(1);
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  millis_ += 1;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  millis_ += 8;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0);

  millis_ += 1;
  pid.run(0);
  EXPECT_FLOAT_EQ(pid.getOutput(), 0.1);
}

};  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // if you plan to use GMock, replace the line above with
  // ::testing::InitGoogleMock(&argc, argv);

  if (RUN_ALL_TESTS())
    ;

  // Always return zero-code and allow PlatformIO to parse results
  return 0;
}

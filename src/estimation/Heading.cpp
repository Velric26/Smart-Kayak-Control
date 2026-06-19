#include "estimation/Heading.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

static float wrap360(float a) { while (a >= 360.f) a -= 360.f; while (a < 0.f) a += 360.f; return a; }
static float wrapDiff(float d) { while (d > 180.f) d -= 360.f; while (d < -180.f) d += 360.f; return d; }

static float compassFrom(IMU* imu, const float off[3], const float scale[3]) {
  float mx = (imu->mRaw[0] - off[0]) * scale[0];
  float my = (imu->mRaw[1] - off[1]) * scale[1];
  float h = atan2f(my, mx) * 57.2957795f;
  return wrap360(h);
}

void HeadingAHRS::begin(IMU* imu) {
  imu_ = imu;

  // Gyro-Z zero-rate bias: average while still.
  double s = 0;
  const int N = 400;
  for (int i = 0; i < N; ++i) { imu_->readGyro(); s += imu_->g[2]; delay(3); }
  gzBias_ = (float)(s / N);

  // Seed the fused heading with the current compass reading.
  imu_->readMag();
  compass_ = compassFrom(imu_, magOff, magScale);
  h_ = compass_;
  init_ = true;
}

void HeadingAHRS::update(float dt) {
  if (!init_) return;
  imu_->readGyro();
  imu_->readMag();

  float gz = (imu_->g[2] - gzBias_) * GYRO_YAW_SIGN;   // deg/s about vertical
  compass_ = compassFrom(imu_, magOff, magScale);

  float predicted = h_ + gz * dt;                       // gyro prediction
  float err = wrapDiff(compass_ - predicted);           // shortest error toward compass
  h_ = wrap360(predicted + (1.0f - HEADING_FUSE_ALPHA) * err);
}

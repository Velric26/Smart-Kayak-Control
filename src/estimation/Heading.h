#pragma once
#include "estimation/IMU.h"

// Heading estimator: complementary filter fusing gyro-Z (smooth, drifts)
// with the calibrated magnetic compass (absolute, noisy). Output is a
// stable heading in degrees [0,360). Tilt compensation is added later;
// this version assumes a roughly level vehicle (fine for rover + calm water).
class HeadingAHRS {
public:
  void  begin(IMU* imu);     // measures gyro-Z bias (hold still ~1.5 s), seeds heading
  void  update(float dt);    // reads IMU and advances the fused heading
  float deg() const { return h_; }         // fused heading
  float compassDeg() const { return compass_; }  // instantaneous compass (debug)

private:
  IMU*  imu_ = nullptr;
  float gzBias_ = 0;
  float h_ = 0, compass_ = 0;
  bool  init_ = false;
};

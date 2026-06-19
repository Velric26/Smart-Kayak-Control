#pragma once
#include "estimation/IMU.h"
#include "config.h"

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

  // Live magnetometer calibration override (from `cal compass` / NVS). Defaults
  // to the config constants; corrected axis = (raw - off) * scale.
  void setMagCal(const float off[3], const float scale[3]) {
    for (int i = 0; i < 3; ++i) { magOff[i] = off[i]; magScale[i] = scale[i]; }
  }
  float magOff[3]   = {MAG_OFF[0],   MAG_OFF[1],   MAG_OFF[2]};
  float magScale[3] = {MAG_SCALE[0], MAG_SCALE[1], MAG_SCALE[2]};

private:
  IMU*  imu_ = nullptr;
  float gzBias_ = 0;
  float h_ = 0, compass_ = 0;
  bool  init_ = false;
};

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

  // Fused heading, with the north-alignment offset applied + wrapped to [0,360).
  // The offset reconciles the compass frame with true north (GPS) for ANCHOR;
  // it cancels out of HEADING_HOLD (relative), so it only affects absolute use.
  float deg() const {
    float v = h_ + headingOffset_;
    while (v >= 360.0f) v -= 360.0f;
    while (v <    0.0f) v += 360.0f;
    return v;
  }
  float compassDeg() const { return compass_; }  // instantaneous compass (debug)

  void  setHeadingOffset(float d) { headingOffset_ = d; }
  float headingOffset() const { return headingOffset_; }

  // Complementary-filter gyro trust per step (live "fuse"): higher = smoother
  // but laggier; lower = snappier pull to the absolute compass but noisier.
  float fuseAlpha = HEADING_FUSE_ALPHA;

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
  float headingOffset_ = 0;   // compass->true-north trim (deg), from `hoff`/NVS
  bool  init_ = false;
};

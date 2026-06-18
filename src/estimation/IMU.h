#pragma once
#include <Arduino.h>

// 9-DOF IMU driver: L3G4200D gyro (0x69) + LSM303DLHC accel (0x19) / mag (0x1E).
// Register-level, no external libraries. Phase 2 (raw + calibration + fusion).
class IMU {
public:
  bool begin();        // configures all three; false if gyro WHO_AM_I != 0xD3
  void readGyro();     // -> g[]  in deg/s
  void readAccel();    // -> a[]  in g
  void readMag();      // -> mRaw[] in raw counts (calibrated in a later step)

  float   g[3]    = {0, 0, 0};   // gyro  x,y,z  (deg/s)
  float   a[3]    = {0, 0, 0};   // accel x,y,z  (g)
  int16_t mRaw[3] = {0, 0, 0};   // mag   x,y,z  (raw)

private:
  static constexpr uint8_t ADDR_GYRO  = 0x69;
  static constexpr uint8_t ADDR_ACCEL = 0x19;
  static constexpr uint8_t ADDR_MAG   = 0x1E;
};

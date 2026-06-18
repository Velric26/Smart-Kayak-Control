#pragma once
#include <Arduino.h>

// 9-DOF IMU on the GY-801-style board (raw register drivers, no external libs):
//   L3G4200D    gyro  @ 0x69   (WHO_AM_I 0xD3)
//   LSM303DLHC  accel @ 0x19
//   LSM303DLHC  mag   @ 0x1E   (note: X,Z,Y order, big-endian)
// Outputs in physical units; the AHRS (Phase 2b) consumes these.
class IMU {
public:
  bool begin();                                      // init all 3; true if gyro ID ok
  void readGyro (float& gx, float& gy, float& gz);   // deg/s
  void readAccel(float& ax, float& ay, float& az);   // g
  void readMag  (float& mx, float& my, float& mz);   // gauss (direction is what matters)

  static constexpr uint8_t ADDR_GYRO  = 0x69;
  static constexpr uint8_t ADDR_ACCEL = 0x19;
  static constexpr uint8_t ADDR_MAG   = 0x1E;
};

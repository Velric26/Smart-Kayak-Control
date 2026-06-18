// =====================================================================
//  diag/imu_raw.cpp  -  raw IMU readout (Phase 2 sanity check).
//  Build/run:  pio run -e diag_imu -t upload && pio device monitor
//
//  Sanity checks with the board sitting flat and still:
//    * gyro  G ~ 0 on all axes (a few deg/s of noise is fine)
//    * accel one axis ~ +/-1.0 g (gravity), the other two near 0
//    * mag   M values change as you rotate the board in the horizontal plane
//  Pick the board up and rotate it: the gyro axis you spin about should
//  read a clear +/- value, then return to ~0 when you stop.
// =====================================================================
#include <Arduino.h>
#include "estimation/IMU.h"

IMU imu;

void setup() {
  Serial.begin(115200);
  delay(300);
  if (!imu.begin()) {
    Serial.println("IMU begin FAILED - gyro WHO_AM_I != 0xD3. Check wiring/power.");
  } else {
    Serial.println("IMU OK.  G=deg/s  A=g  M=raw");
  }
}

void loop() {
  imu.readGyro();
  imu.readAccel();
  imu.readMag();
  Serial.printf("G[%+7.1f %+7.1f %+7.1f]  A[%+5.2f %+5.2f %+5.2f]  M[%+6d %+6d %+6d]\n",
                imu.g[0], imu.g[1], imu.g[2],
                imu.a[0], imu.a[1], imu.a[2],
                imu.mRaw[0], imu.mRaw[1], imu.mRaw[2]);
  delay(100);
}

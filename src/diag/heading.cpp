// =====================================================================
//  diag/heading.cpp  -  flat magnetic heading (Phase 2c, step 1).
//  Build/run:  pio run -e diag_heading -t upload && pio device monitor
//
//  Uses MAG_OFF / MAG_SCALE from config.h, so paste your calibration in
//  first. Keep the board LEVEL and rotate it slowly: heading should sweep
//  0..360 smoothly and in ONE direction (no jumps, no doubling back).
//
//  This is a 2D compass (no tilt comp, no gyro yet) - the simplest thing
//  that proves the calibrated mag gives a correct bearing. Once the sweep
//  looks clean we add tilt compensation + Mahony gyro fusion.
// =====================================================================
#include <Arduino.h>
#include <math.h>
#include "estimation/IMU.h"
#include "config.h"

IMU imu;

void setup() {
  Serial.begin(115200);
  delay(300);
  if (!imu.begin()) { Serial.println("IMU begin FAILED"); while (1) delay(1000); }
  Serial.println("Flat heading. Keep board LEVEL, rotate slowly through 360.");
}

void loop() {
  imu.readMag();
  float mx = (imu.mRaw[0] - MAG_OFF[0]) * MAG_SCALE[0];
  float my = (imu.mRaw[1] - MAG_OFF[1]) * MAG_SCALE[1];

  float h = atan2f(my, mx) * 57.2957795f;   // sign/axis convention TBD on the bench
  if (h < 0) h += 360.0f;

  Serial.printf("mx=%+7.1f  my=%+7.1f   heading=%6.1f deg\n", mx, my, h);
  delay(100);
}

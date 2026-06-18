// =====================================================================
//  diag/imu_calib.cpp  -  IMU calibration (Phase 2b).
//  Build/run:  pio run -e diag_calib -t upload && pio device monitor
//
//  1) Hold the board STILL ~2 s  -> prints gyro bias (deg/s).
//  2) Then slowly ROTATE/TUMBLE the board through ALL orientations for
//     ~25 s (every face up, full flat spins) while it tracks mag min/max.
//  When the min/max stop growing, copy the printed MAG_OFF / MAG_SCALE
//  lines into include/config.h.
//
//  Watch Z: if its min/max barely spread while X/Y swing widely, the Z
//  channel isn't seeing field and we need to look at it.
// =====================================================================
#include <Arduino.h>
#include "estimation/IMU.h"

IMU imu;
bool ok = false;
int16_t mn[3], mx[3];

void setup() {
  Serial.begin(115200);
  delay(300);
  ok = imu.begin();
  if (!ok) { Serial.println("IMU begin FAILED - check wiring."); return; }

  // ---- gyro bias: average while still ----
  Serial.println("Hold the board STILL...");
  delay(1000);
  double s[3] = {0, 0, 0};
  const int N = 500;
  for (int i = 0; i < N; ++i) { imu.readGyro(); s[0]+=imu.g[0]; s[1]+=imu.g[1]; s[2]+=imu.g[2]; delay(4); }
  Serial.printf("\n// gyro bias (informational; the AHRS re-zeros at boot)\n");
  Serial.printf("GYRO_BIAS dps: X=%.3f Y=%.3f Z=%.3f\n\n",
                s[0]/N, s[1]/N, s[2]/N);

  imu.readMag();
  for (int k = 0; k < 3; ++k) { mn[k] = mx[k] = imu.mRaw[k]; }
  Serial.println("Now ROTATE/TUMBLE through ALL orientations (~25 s)...");
}

void loop() {
  if (!ok) { delay(1000); return; }
  imu.readMag();
  for (int k = 0; k < 3; ++k) {
    if (imu.mRaw[k] < mn[k]) mn[k] = imu.mRaw[k];
    if (imu.mRaw[k] > mx[k]) mx[k] = imu.mRaw[k];
  }
  static uint32_t t = 0;
  if (millis() - t > 500) {
    t = millis();
    float off[3], rad[3], ravg = 0;
    for (int k = 0; k < 3; ++k) {
      off[k] = (mx[k] + mn[k]) * 0.5f;
      rad[k] = (mx[k] - mn[k]) * 0.5f;
      ravg += rad[k];
    }
    ravg /= 3.0f;
    float sc[3];
    for (int k = 0; k < 3; ++k) sc[k] = (rad[k] > 1.0f) ? (ravg / rad[k]) : 1.0f;

    Serial.printf("min[%5d %5d %5d] max[%5d %5d %5d]\n", mn[0],mn[1],mn[2], mx[0],mx[1],mx[2]);
    Serial.printf("  -> constexpr float MAG_OFF[3]   = {%.1ff, %.1ff, %.1ff};\n", off[0],off[1],off[2]);
    Serial.printf("  -> constexpr float MAG_SCALE[3] = {%.3ff, %.3ff, %.3ff};\n\n", sc[0],sc[1],sc[2]);
  }
}

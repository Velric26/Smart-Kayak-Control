// =====================================================================
//  diag/heading_fused.cpp  -  compass vs gyro-fused heading (Phase 2c, step 2).
//  Build/run:  pio run -e diag_fused -t upload && pio device monitor
//
//  Hold the board STILL at startup (~1.5 s) while it learns the gyro bias.
//  Then rotate it:
//    * fused should track the turn SMOOTHLY and with less noise than compass
//    * fused and compass should agree when you stop
//    * spin quickly: fused responds instantly (gyro), then settles to compass
//  If fused runs the OPPOSITE way from compass when you rotate, flip
//  GYRO_YAW_SIGN to -1.0 in config.h.
//  (No tilt compensation yet - keep it roughly level for this test.)
// =====================================================================
#include <Arduino.h>
#include "estimation/IMU.h"
#include "estimation/Heading.h"

IMU imu;
HeadingAHRS ahrs;

void setup() {
  Serial.begin(115200);
  delay(300);
  if (!imu.begin()) { Serial.println("IMU begin FAILED"); while (1) delay(1000); }
  Serial.println("Hold STILL for gyro bias...");
  delay(500);
  ahrs.begin(&imu);
  Serial.println("OK. Rotate to compare compass vs fused.");
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = micros();
  float dt = (now - last) * 1e-6f;
  last = now;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;

  ahrs.update(dt);

  static uint32_t t = 0;
  if (millis() - t >= 100) {
    t = millis();
    Serial.printf("compass=%6.1f   fused=%6.1f   deg\n", ahrs.compassDeg(), ahrs.deg());
  }
  delay(8);   // ~100 Hz including I2C read time
}

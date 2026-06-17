// =====================================================================
//  diag/i2c_scan.cpp  -  standalone I2C bus scanner (Phase 2 bring-up).
//  Build/run with:   pio run -e diag_i2c -t upload && pio device monitor
//  Confirms the IMU chips answer on the bus BEFORE we add any sensor
//  library or write the AHRS. Uses the project's I2C pins from config.h.
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

static const char* knownName(uint8_t a) {
  switch (a) {
    case 0x6B:
    case 0x6A: return "L3GD20(H) gyro";
    case 0x69:
    case 0x68: return "gyro (L3G4200D / MPU-class)";
    case 0x19: return "LSM303DLHC accel";
    case 0x1E: return "LSM303DLHC mag";
    case 0x77: return "BMP180 baro/temp";
    default:   return "(unknown)";
  }
}

// Single-register read (STM sub-address, MSB=0 for a one-byte read).
static uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((int)addr, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// Probe the gyro's WHO_AM_I (reg 0x0F) to identify the exact chip.
static void identifyGyro() {
  const uint8_t cands[] = {0x69, 0x68, 0x6A, 0x6B};
  for (uint8_t a : cands) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    uint8_t who = readReg(a, 0x0F);
    const char* id = (who == 0xD3) ? "L3G4200D" :
                     (who == 0xD4) ? "L3GD20"   :
                     (who == 0xD7) ? "L3GD20H"  : "unrecognized";
    Serial.printf("  gyro @0x%02X  WHO_AM_I=0x%02X -> %s\n", a, who, id);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  Serial.printf("\nI2C scanner  SDA=GPIO%d  SCL=GPIO%d\n", PIN_I2C_SDA, PIN_I2C_SCL);
  Serial.println("Expecting: 0x6B(/0x6A) gyro, 0x19 accel, 0x1E mag, 0x77 BMP180");
}

void loop() {
  Serial.println("Scanning...");
  int n = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X  %s\n", a, knownName(a));
      ++n;
    }
  }
  if (n == 0)
    Serial.println("  none found - check 3V3, GND, SDA/SCL not swapped, pull-ups");
  else
    Serial.printf("  %d device(s) found\n", n);
  identifyGyro();
  Serial.println();
  delay(2000);
}

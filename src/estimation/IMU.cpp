#include "estimation/IMU.h"
#include <Wire.h>
#include "config.h"

// ---------------- low-level I2C helpers ----------------
static void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
static uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr); Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, 1);
  return Wire.available() ? Wire.read() : 0;
}
static void readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr); Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, (int)n);
  for (uint8_t i = 0; i < n && Wire.available(); ++i) buf[i] = Wire.read();
}

// ---------------- scale factors ----------------
static constexpr float GYRO_DPS_PER_LSB = 0.0175f; // 500 dps full scale
static constexpr float ACC_G_PER_LSB    = 0.001f;  // +-2g, high-res 12-bit (1 mg/LSB)

bool IMU::begin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  // L3G4200D gyro
  if (readReg(ADDR_GYRO, 0x0F) != 0xD3) return false;   // WHO_AM_I
  writeReg(ADDR_GYRO, 0x20, 0x0F);   // CTRL_REG1: power on, XYZ enabled, 100 Hz
  writeReg(ADDR_GYRO, 0x23, 0x90);   // CTRL_REG4: BDU, 500 dps

  // LSM303DLHC accelerometer
  writeReg(ADDR_ACCEL, 0x20, 0x57);  // CTRL_REG1_A: 100 Hz, XYZ enabled
  writeReg(ADDR_ACCEL, 0x23, 0x08);  // CTRL_REG4_A: high-res, +-2g

  // LSM303DLHC magnetometer
  writeReg(ADDR_MAG, 0x00, 0x18);    // CRA_REG_M: 75 Hz output
  writeReg(ADDR_MAG, 0x01, 0x20);    // CRB_REG_M: +-1.3 gauss range
  writeReg(ADDR_MAG, 0x02, 0x00);    // MR_REG_M: continuous-conversion mode
  return true;
}

void IMU::readGyro() {
  uint8_t b[6];
  readRegs(ADDR_GYRO, 0x28 | 0x80, b, 6);   // 0x80 = auto-increment, little-endian
  int16_t x = (int16_t)((b[1] << 8) | b[0]);
  int16_t y = (int16_t)((b[3] << 8) | b[2]);
  int16_t z = (int16_t)((b[5] << 8) | b[4]);
  g[0] = x * GYRO_DPS_PER_LSB;
  g[1] = y * GYRO_DPS_PER_LSB;
  g[2] = z * GYRO_DPS_PER_LSB;
}

void IMU::readAccel() {
  uint8_t b[6];
  readRegs(ADDR_ACCEL, 0x28 | 0x80, b, 6);  // auto-increment, little-endian
  int16_t x = (int16_t)((b[1] << 8) | b[0]);
  int16_t y = (int16_t)((b[3] << 8) | b[2]);
  int16_t z = (int16_t)((b[5] << 8) | b[4]);
  // HR mode: 12-bit left-justified -> shift right 4
  a[0] = (x >> 4) * ACC_G_PER_LSB;
  a[1] = (y >> 4) * ACC_G_PER_LSB;
  a[2] = (z >> 4) * ACC_G_PER_LSB;
}

void IMU::readMag() {
  uint8_t b[6];
  // DLHC mag quirk: register order is X_H,X_L, Z_H,Z_L, Y_H,Y_L (big-endian,
  // Z before Y), and it auto-increments without the 0x80 flag.
  readRegs(ADDR_MAG, 0x03, b, 6);
  mRaw[0] = (int16_t)((b[0] << 8) | b[1]); // X
  mRaw[2] = (int16_t)((b[2] << 8) | b[3]); // Z
  mRaw[1] = (int16_t)((b[4] << 8) | b[5]); // Y
}

#include "hal/PowerMonitor.h"
#include <Wire.h>

// ---- INA228 register map (datasheet section 7.6) ----
static constexpr uint8_t REG_CONFIG     = 0x00;  // RST bit15, ADCRANGE bit4
static constexpr uint8_t REG_ADC_CONFIG = 0x01;  // MODE/conv-times/averaging
static constexpr uint8_t REG_SHUNT_CAL  = 0x02;
static constexpr uint8_t REG_VBUS       = 0x05;  // 24-bit, [23:4], 195.3125 uV/LSB
static constexpr uint8_t REG_DIETEMP    = 0x06;  // 16-bit, 7.8125 mdegC/LSB
static constexpr uint8_t REG_CURRENT    = 0x07;  // 24-bit signed, [23:4], CURRENT_LSB
static constexpr uint8_t REG_POWER      = 0x08;  // 24-bit, 3.2 * CURRENT_LSB W/LSB
static constexpr uint8_t REG_DEVICE_ID  = 0x3F;  // 0x228 in bits [15:4]

bool PowerMonitor::begin(uint8_t addr, float shuntOhms, float maxAmps) {
  addr_ = addr;
  // Identify the chip before configuring anything.
  uint16_t id = (uint16_t)readReg(REG_DEVICE_ID, 2);
  if ((id >> 4) != 0x228) { ok_ = false; return false; }

  writeReg16(REG_CONFIG, 0x8000);            // soft reset
  delay(2);
  // ADCRANGE=0: +-163.84 mV shunt range. At R002/20A that's 40 mV max drop —
  // plenty of headroom; range 1 (+-40.96 mV) could be revisited for resolution.
  writeReg16(REG_CONFIG, 0x0000);
  // Continuous bus+shunt+temp, 1052 us conversions, 16-sample averaging:
  // MODE=0xF, VBUSCT=VSHCT=VTCT=0b101, AVG=0b010.
  writeReg16(REG_ADC_CONFIG, (0xFu << 12) | (5u << 9) | (5u << 6) | (5u << 3) | 2u);

  // CURRENT_LSB = maxAmps / 2^19; SHUNT_CAL = 13107.2e6 * CURRENT_LSB * Rshunt.
  currentLsb_ = maxAmps / 524288.0f;
  writeReg16(REG_SHUNT_CAL, (uint16_t)(13107.2e6f * currentLsb_ * shuntOhms));
  ok_ = true;
  return true;
}

float PowerMonitor::volts() {
  return (readReg(REG_VBUS, 3) >> 4) * 195.3125e-6f;
}

float PowerMonitor::amps() {
  int32_t raw = (int32_t)(readReg(REG_CURRENT, 3) << 8) >> 12;  // sign-extend [23:4]
  return raw * currentLsb_;
}

float PowerMonitor::watts() {
  return readReg(REG_POWER, 3) * 3.2f * currentLsb_;
}

float PowerMonitor::dieTempC() {
  return (int16_t)readReg(REG_DIETEMP, 2) * 7.8125e-3f;
}

uint32_t PowerMonitor::readReg(uint8_t reg, uint8_t bytes) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom((int)addr_, (int)bytes);
  uint32_t v = 0;
  for (uint8_t i = 0; i < bytes && Wire.available(); ++i)
    v = (v << 8) | Wire.read();                 // big-endian
  return v;
}

void PowerMonitor::writeReg16(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  Wire.endTransmission();
}

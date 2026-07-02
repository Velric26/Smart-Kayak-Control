#pragma once
#include <Arduino.h>

// TI INA228 high-precision I2C power monitor (20-bit V/I, on-chip P/E).
// One instance per device: the plan (config.h) is R002 on the motor battery
// and R015 on the logic rail, superseding the resistor-divider battery plan.
//
// ⚠ STATUS: written ahead of the hardware — compiles, but UNVALIDATED until
// the modules arrive. Bring-up: `pio run -e diag_power -t upload`, confirm
// the addresses with diag_i2c first.
//
// Usage:
//   PowerMonitor battMon;
//   battMon.begin(INA_BATT_ADDR, INA_BATT_SHUNT, INA_BATT_MAX_A);
//   float v = battMon.volts(), a = battMon.amps(), w = battMon.watts();
//
// Reads are on-demand over I2C (~0.1 ms each at 400 kHz); the chip free-runs
// continuous conversions in the background, so values are always fresh.
class PowerMonitor {
public:
  // maxAmps sets the current LSB (maxAmps / 2^19) and the SHUNT_CAL value —
  // pick the smallest full-scale that covers your peaks for best resolution.
  // Returns false if the chip doesn't answer or the DEVICE_ID is wrong.
  bool begin(uint8_t addr, float shuntOhms, float maxAmps);

  bool  ok() const { return ok_; }
  float volts();     // bus voltage (V)
  float amps();      // shunt current (A, signed)
  float watts();     // power (W)
  float dieTempC();  // internal die temperature (deg C)

private:
  uint32_t readReg(uint8_t reg, uint8_t bytes);
  void     writeReg16(uint8_t reg, uint16_t val);

  uint8_t addr_ = 0;
  float   currentLsb_ = 0;   // A/bit, from maxAmps
  bool    ok_ = false;
};

// =====================================================================
//  diag/power_mon.cpp  -  INA228 power-monitor bring-up (standalone).
//  Build/run:  pio run -e diag_power -t upload && pio device monitor
//
//  Expects the two planned units (config.h): battery R002 @ INA_BATT_ADDR,
//  logic R015 @ INA_LOGIC_ADDR. Run diag_i2c FIRST to confirm the strapped
//  addresses; a missing unit is reported and simply skipped.
//  Sanity checks on bring-up: bus volts ~= multimeter, ~0 A with loads off,
//  current sign positive when discharging (swap shunt leads if inverted).
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "hal/PowerMonitor.h"

PowerMonitor battMon, logicMon;
bool haveBatt = false, haveLogic = false;

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  Serial.println("\nINA228 bring-up");
  haveBatt  = battMon.begin(INA_BATT_ADDR, INA_BATT_SHUNT, INA_BATT_MAX_A);
  haveLogic = logicMon.begin(INA_LOGIC_ADDR, INA_LOGIC_SHUNT, INA_LOGIC_MAX_A);
  Serial.printf("  battery (0x%02X, R%03.0f): %s\n", INA_BATT_ADDR,
                INA_BATT_SHUNT * 1000, haveBatt ? "OK" : "NOT FOUND");
  Serial.printf("  logic   (0x%02X, R%03.0f): %s\n", INA_LOGIC_ADDR,
                INA_LOGIC_SHUNT * 1000, haveLogic ? "OK" : "NOT FOUND");
  if (!haveBatt && !haveLogic)
    Serial.println("  none answered - check wiring/addresses with diag_i2c");
}

void loop() {
  if (haveBatt)
    Serial.printf("batt : %6.3f V  %+7.3f A  %7.3f W  (die %.1f C)\n",
                  battMon.volts(), battMon.amps(), battMon.watts(), battMon.dieTempC());
  if (haveLogic)
    Serial.printf("logic: %6.3f V  %+7.3f A  %7.3f W\n",
                  logicMon.volts(), logicMon.amps(), logicMon.watts());
  delay(500);
}

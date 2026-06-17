#include "hal/BatteryMonitor.h"
#include "config.h"

void BatteryMonitor::begin() {
  pinMode(PIN_BATT_SENSE, INPUT);
  update();
}

void BatteryMonitor::update() {
  // analogReadMilliVolts applies the per-chip ADC calibration curve.
  uint32_t mv = 0;
  const int N = 8;
  for (int i = 0; i < N; ++i) mv += analogReadMilliVolts(PIN_BATT_SENSE);
  float vadc = (mv / (float)N) / 1000.0f;
  float v = vadc * BATT_DIVIDER;
  // Light IIR smoothing.
  v_ = 0.8f * v_ + 0.2f * v;
}

bool BatteryMonitor::warn() const     { return v_ <= BATT_WARN_V; }
bool BatteryMonitor::critical() const { return v_ <= BATT_CRITICAL_V; }

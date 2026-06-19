#include "hal/GPS.h"
#include "config.h"

// Set by tools/gps_config.py (5 Hz + GST needs more than 9600 can carry).
// If you ever factory-reset the module, it reverts to 9600 -> set this back.
static const uint32_t GPS_BAUD = 38400;

void GPS::begin() {
  // UART2: RX=PIN_GPS_RX (<- GPS TX), TX=PIN_GPS_TX (-> GPS RX).
  Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  // GST stdLat/stdLon (terms 6/7) -> horizontal accuracy estimate.
  gstLat_.begin(gps_, "GNGST", 6);
  gstLon_.begin(gps_, "GNGST", 7);
}

void GPS::update() {
  while (Serial2.available()) gps_.encode(Serial2.read());
}

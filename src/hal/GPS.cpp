#include "hal/GPS.h"
#include "config.h"

static const uint32_t GPS_BAUD = 9600;   // NEO-8M factory default

void GPS::begin() {
  // UART2: RX=PIN_GPS_RX (<- GPS TX), TX=PIN_GPS_TX (-> GPS RX).
  Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
}

void GPS::update() {
  while (Serial2.available()) gps_.encode(Serial2.read());
}

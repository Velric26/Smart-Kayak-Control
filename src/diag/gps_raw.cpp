// =====================================================================
//  diag/gps_raw.cpp  -  standalone NEO-8M bring-up (Phase 4).
//  Build/run with:   pio run -e diag_gps -t upload && pio device monitor
//  Confirms the GPS talks on UART2 and reports fix quality BEFORE we add
//  the Position estimator or ANCHOR mode. Uses the reserved GPS pins from
//  config.h (RX=GPIO16 <- GPS TX, TX=GPIO17 -> GPS RX) at 9600 baud.
//
//  What to look for, in order:
//   1. "bytes" climbing      -> wiring + baud OK (NMEA is arriving at all).
//      If it stays 0: GPS TX -> ESP32 RX(16) not connected, or wrong baud,
//      or GPS unpowered. (RX/TX swap is the usual culprit.)
//   2. "chars/sentences" in the TinyGPS stats climbing, fail=0 -> parsing OK.
//   3. "sats" rising, then a "FIX" with lat/lon -> needs a clear sky view;
//      a cold start outdoors can take 30-90 s. Indoors it may never fix.
// =====================================================================
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <TinyGPSPlus.h>
#include "config.h"

static const uint32_t GPS_BAUD = 9600;   // NEO-8M factory default

TinyGPSPlus    gps;
BluetoothSerial SerialBT;                 // mirror so you can roam off-USB for sky view
uint32_t       rxBytes = 0;

// Echo a line to both USB and Bluetooth.
static void echo(const char* s) { Serial.print(s); SerialBT.print(s); }

void setup() {
  Serial.begin(115200);
  delay(300);
  SerialBT.begin(BT_DEVICE_NAME);
  // UART2: RX=PIN_GPS_RX (from GPS TX), TX=PIN_GPS_TX (to GPS RX).
  Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  char m[140];
  snprintf(m, sizeof(m),
           "\r\nGPS diag  UART2 RX=GPIO%d TX=GPIO%d  @%lu baud  (BT: %s)\r\n"
           "Waiting for NMEA... (put the antenna by a window / outdoors)\r\n",
           PIN_GPS_RX, PIN_GPS_TX, (unsigned long)GPS_BAUD, BT_DEVICE_NAME);
  echo(m);
}

void loop() {
  // Pump every available byte into the parser.
  while (Serial2.available()) {
    char c = Serial2.read();
    gps.encode(c);
    rxBytes++;
  }

  // Status line once a second.
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    char m[200];

    if (rxBytes == 0) {
      echo("  bytes=0  -> NO data. Check GPS TX -> ESP32 GPIO16, GND, 3V3, baud.\r\n");
      return;
    }

    snprintf(m, sizeof(m),
             "  bytes=%lu  rx_chars=%lu sentences=%lu checksum_fail=%lu  sats=%d hdop=%.1f  ",
             (unsigned long)rxBytes,
             (unsigned long)gps.charsProcessed(),
             (unsigned long)gps.sentencesWithFix(),
             (unsigned long)gps.failedChecksum(),
             gps.satellites.isValid() ? gps.satellites.value() : 0,
             gps.hdop.isValid() ? gps.hdop.hdop() : 0.0);
    echo(m);

    if (gps.location.isValid()) {
      snprintf(m, sizeof(m), "FIX lat=%.6f lon=%.6f alt=%.1fm age=%lums\r\n",
               gps.location.lat(), gps.location.lng(),
               gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
               (unsigned long)gps.location.age());
      echo(m);
    } else {
      echo("no fix yet (acquiring)...\r\n");
    }

    // A live-but-unparsed stream (bytes climbing, rx_chars stuck) usually means
    // a baud mismatch -> the bytes are framing garbage.
    if (rxBytes > 1000 && gps.charsProcessed() < 10)
      echo("  !! data arriving but not parsing -> wrong baud (try 38400)?\r\n");
  }
}

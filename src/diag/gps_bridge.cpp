// =====================================================================
//  diag/gps_bridge.cpp  -  transparent USB <-> GPS UART2 pass-through.
//  Lets you (re)configure the NEO-8M with tools/gps_config.py WITHOUT a
//  separate USB-UART adapter: the ESP32 just relays bytes both ways.
//
//  Use:
//    1. pio run -e diag_gps_bridge -t upload
//    2. close the PlatformIO monitor, then:
//         python tools/gps_config.py COMxx 38400      (COMxx = the ESP32 port)
//    3. pio run -e mule -t upload   (flash the real firmware back)
//
//  Both links run at 38400 to match the module's saved baud. If the module
//  was factory-reset (back to 9600), change BRIDGE_BAUD to 9600, reconfigure,
//  then it'll be 38400 again. No data shaping — pure byte relay, so UBX ACKs
//  and the baud-change handshake pass through untouched.
// =====================================================================
#include <Arduino.h>
#include "config.h"

static const uint32_t BRIDGE_BAUD = 38400;

void setup() {
  Serial.begin(BRIDGE_BAUD);                                  // USB <-> PC
  Serial2.begin(BRIDGE_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX); // <-> GPS
}

void loop() {
  while (Serial.available())  Serial2.write(Serial.read());   // PC -> GPS
  while (Serial2.available()) Serial.write(Serial2.read());   // GPS -> PC
}

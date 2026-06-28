// =====================================================================
//  nano_level_shifter.ino  -  Arduino Nano (5V) as a 2-channel 3.3V->5V
//  servo-PWM level shifter between the ESP32 and the dual ESC.
//
//  The ESC needs a 5V-logic servo pulse to arm; the ESP32 only outputs 3.3V.
//  The Nano reads each ESP32 channel (3.3V reads as HIGH on the 5V ATmega),
//  measures the pulse width, and regenerates a clean 5V servo pulse to the
//  ESC. Adds ~one 20 ms frame of latency (fine for an ESC) and fails safe to
//  neutral (1500 us) if the ESP32 signal disappears.
//
//  Wiring (COMMON GROUND IS MANDATORY — tie ESP32 GND, Nano GND, ESC GND):
//    ESP32 GPIO25 (L sig) -> Nano D2   |  Nano D9  -> ESC L signal
//    ESP32 GPIO26 (R sig) -> Nano D3   |  Nano D10 -> ESC R signal
//    Nano 5V from USB or Vin (7-12V); its 5V logic drives the ESCs.
//  D2/D3 are the Nano's hardware-interrupt pins; D9/D10 are the Servo lib pins.
// =====================================================================
#include <Servo.h>

const uint8_t IN_L = 2, IN_R = 3;     // from ESP32 (3.3V)
const uint8_t OUT_L = 9, OUT_R = 10;  // to ESC (5V)

const uint16_t US_MIN = 800, US_MAX = 2200, US_NEUTRAL = 1500;  // ESC legal window
const uint16_t STALE_MS = 100;        // no fresh pulse this long -> neutral

Servo escL, escR;

volatile uint16_t widthL = US_NEUTRAL, widthR = US_NEUTRAL;
volatile uint32_t riseL = 0, riseR = 0;
volatile uint32_t lastL = 0, lastR = 0;

void isrL() {
  if (digitalRead(IN_L)) { riseL = micros(); }
  else { uint32_t w = micros() - riseL; if (w >= US_MIN && w <= US_MAX) { widthL = w; lastL = millis(); } }
}
void isrR() {
  if (digitalRead(IN_R)) { riseR = micros(); }
  else { uint32_t w = micros() - riseR; if (w >= US_MIN && w <= US_MAX) { widthR = w; lastR = millis(); } }
}

void setup() {
  pinMode(IN_L, INPUT);
  pinMode(IN_R, INPUT);
  escL.attach(OUT_L, US_MIN, US_MAX);
  escR.attach(OUT_R, US_MIN, US_MAX);
  escL.writeMicroseconds(US_NEUTRAL);     // hold neutral so the ESC can arm
  escR.writeMicroseconds(US_NEUTRAL);
  attachInterrupt(digitalPinToInterrupt(IN_L), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IN_R), isrR, CHANGE);
}

void loop() {
  // Snapshot the volatiles atomically (16/32-bit reads aren't atomic on AVR).
  noInterrupts();
  uint16_t wl = widthL, wr = widthR;
  uint32_t ll = lastL,  lr = lastR;
  interrupts();

  uint32_t now = millis();
  escL.writeMicroseconds((now - ll < STALE_MS) ? wl : US_NEUTRAL);  // failsafe -> neutral
  escR.writeMicroseconds((now - lr < STALE_MS) ? wr : US_NEUTRAL);
  delay(5);
}

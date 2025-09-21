#include <Arduino.h>

constexpr uint8_t kEncoderPinA = D5;
constexpr uint8_t kEncoderPinB = D6;
constexpr uint8_t kMotorSensePin1 = D2;
constexpr uint8_t kMotorSensePin2 = D3;

constexpr uint16_t kAnalogThreshold = 80;  // Adjust depending on the divider output.
constexpr unsigned long kSenseIntervalMs = 5;

volatile long g_encoderCount = 0;
volatile bool g_motorActive = false;
volatile uint8_t g_lastAB = 0;

unsigned long g_lastSenseTime = 0;
bool g_prevMotorActive = false;

void handleEncoderChange();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    // Wait for the serial monitor (optional timeout for standalone operation).
  }

  pinMode(kEncoderPinA, INPUT_PULLUP);
  pinMode(kEncoderPinB, INPUT_PULLUP);
  pinMode(kMotorSensePin1, INPUT);
  pinMode(kMotorSensePin2, INPUT);

  analogReadResolution(12);  // SAMD21 supports 12-bit ADC readings (0-4095).

  // Initialize encoder state before enabling interrupts.
  g_lastAB = (digitalRead(kEncoderPinA) << 1) | digitalRead(kEncoderPinB);

  attachInterrupt(digitalPinToInterrupt(kEncoderPinA), handleEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinB), handleEncoderChange, CHANGE);
}

void loop() {
  const unsigned long now = millis();
  if (now - g_lastSenseTime >= kSenseIntervalMs) {
    g_lastSenseTime = now;

    const int sense1 = analogRead(kMotorSensePin1);
    const int sense2 = analogRead(kMotorSensePin2);
    const bool motorActiveNow = (sense1 > kAnalogThreshold) || (sense2 > kAnalogThreshold);

    if (motorActiveNow && !g_prevMotorActive) {
      // Motor just started generating voltage: re-sync the encoder state.
      noInterrupts();
      g_lastAB = (digitalRead(kEncoderPinA) << 1) | digitalRead(kEncoderPinB);
      g_motorActive = true;
      interrupts();
    } else if (!motorActiveNow && g_prevMotorActive) {
      // Motor just stopped: freeze counting and report the total.
      long finalCount;
      noInterrupts();
      g_motorActive = false;
      finalCount = g_encoderCount;
      interrupts();

      Serial.print(F("Tong so xung da dem: "));
      Serial.println(finalCount);
    } else {
      // No change in activity state, update the shared flag if needed.
      noInterrupts();
      g_motorActive = motorActiveNow;
      interrupts();
    }

    g_prevMotorActive = motorActiveNow;
  }
}

void handleEncoderChange() {
  if (!g_motorActive) {
    // Keep the last state updated even while inactive to avoid false counts later.
    g_lastAB = (digitalRead(kEncoderPinA) << 1) | digitalRead(kEncoderPinB);
    return;
  }

  const uint8_t currentAB = (digitalRead(kEncoderPinA) << 1) | digitalRead(kEncoderPinB);
  const uint8_t transition = (g_lastAB << 2) | currentAB;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      g_encoderCount++;
      break;
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      g_encoderCount--;
      break;
    default:
      break;
  }

  g_lastAB = currentAB;
}


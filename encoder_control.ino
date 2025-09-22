#include <Arduino.h>

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

constexpr uint8_t PE_PIN       = A4;  // PE input
constexpr uint8_t ENC_A_PIN    = 5;   // Encoder phase A
constexpr uint8_t ENC_B_PIN    = 6;   // Encoder phase B
constexpr uint8_t PW_PIN       = 7;   // PWM output

// Voltage thresholds (volts)
constexpr float PE_LOW_THRESHOLD  = 0.5f;
constexpr float PE_HIGH_THRESHOLD = 1.0f;

// ADC configuration
constexpr float ADC_REF_VOLTAGE   = 3.3f;
constexpr uint8_t ADC_RESOLUTION_BITS = 10;
constexpr int ADC_RESOLUTION_STEPS    = (1 << ADC_RESOLUTION_BITS) - 1;
constexpr float ADC_SCALE = ADC_REF_VOLTAGE / static_cast<float>(ADC_RESOLUTION_STEPS);

// Motion / PWM configuration
constexpr int32_t TARGET_PULSES = 5000;
constexpr uint8_t PWM_LOW_VALUE  = 39;   // ~0.5 V @ 3.3 V supply
constexpr uint8_t PWM_HIGH_VALUE = 210;  // ~2.9 V @ 3.3 V supply

constexpr uint32_t STATUS_REPORT_INTERVAL_MS = 250;
constexpr uint32_t MOTION_IDLE_TIMEOUT_MS    = 150;

// Encoder state shared with ISRs
volatile int32_t encoderCount      = 0;
volatile int8_t lastStepDirection  = 0;
volatile bool countingEnabled      = false;
volatile uint8_t lastEncoderState  = 0;

// Main loop state
bool pwIsLow                        = false;
uint32_t lastStatusMillis           = 0;
uint32_t lastMovementMillis         = 0;
int32_t previousCountSnapshot       = 0;
float lastPeVoltage                 = 0.0f;

enum class PeState : uint8_t { Inactive, Active };
enum class MotionDirection : int8_t { Idle = 0, Forward = 1, Backward = -1 };

PeState peState                     = PeState::Inactive;
MotionDirection lastMotionDirection = MotionDirection::Idle;

inline uint8_t readEncoderPins() {
  return static_cast<uint8_t>((digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN));
}

void applyPwHigh(const __FlashStringHelper* message = nullptr) {
  analogWrite(PW_PIN, PWM_HIGH_VALUE);
  const bool stateChanged = pwIsLow;
  pwIsLow = false;

  if (message != nullptr) {
    Serial.println(message);
  } else if (stateChanged) {
    Serial.println(F("PW → HIGH"));
  }
}

void applyPwLow(const __FlashStringHelper* message = nullptr) {
  analogWrite(PW_PIN, PWM_LOW_VALUE);
  const bool stateChanged = !pwIsLow;
  pwIsLow = true;

  if (message != nullptr) {
    Serial.println(message);
  } else if (stateChanged) {
    Serial.println(F("PW → LOW"));
  }
}

void transitionToInactive() {
  noInterrupts();
  countingEnabled     = false;
  encoderCount        = 0;
  lastStepDirection   = 0;
  lastEncoderState    = readEncoderPins();
  interrupts();

  previousCountSnapshot = 0;
  lastMotionDirection   = MotionDirection::Idle;
  lastMovementMillis    = 0;

  applyPwHigh(F("🔼 PE ≥ 1.0V → reset counters, PW = HIGH"));
  peState = PeState::Inactive;
}

void transitionToActive() {
  noInterrupts();
  encoderCount        = 0;
  lastStepDirection   = 0;
  lastEncoderState    = readEncoderPins();
  countingEnabled     = true;
  interrupts();

  previousCountSnapshot = 0;
  lastMotionDirection   = MotionDirection::Idle;
  lastMovementMillis    = millis();

  applyPwHigh(F("🔽 PE < 0.5V → counting enabled, PW = HIGH"));
  peState = PeState::Active;
}

float readPeVoltage() {
  const int raw = analogRead(PE_PIN);
  return raw * ADC_SCALE;
}

const __FlashStringHelper* directionLabel(MotionDirection direction) {
  switch (direction) {
    case MotionDirection::Forward:
      return F("Forward");
    case MotionDirection::Backward:
      return F("Backward");
    default:
      return F("Idle");
  }
}

void reportStatus(int32_t countSnapshot, MotionDirection instantaneousDirection) {
  const uint32_t now = millis();
  if (now - lastStatusMillis < STATUS_REPORT_INTERVAL_MS) {
    return;
  }

  lastStatusMillis = now;

  MotionDirection displayDirection = instantaneousDirection;
  if (displayDirection == MotionDirection::Idle &&
      (now - lastMovementMillis) <= MOTION_IDLE_TIMEOUT_MS) {
    displayDirection = lastMotionDirection;
  }

  Serial.print(F("Encoder: "));
  Serial.print(countSnapshot);
  Serial.print(F(" | Direction: "));
  Serial.print(directionLabel(displayDirection));
  Serial.print(F(" | PW: "));
  Serial.print(pwIsLow ? F("LOW") : F("HIGH"));
  Serial.print(F(" | PE: "));
  Serial.print(lastPeVoltage, 2);
  Serial.println(F(" V"));
}

void managePwOutput() {
  int32_t countSnapshot;
  int8_t stepDirection;

  noInterrupts();
  countSnapshot = encoderCount;
  stepDirection = lastStepDirection;
  interrupts();

  MotionDirection motion = MotionDirection::Idle;
  if (countSnapshot > previousCountSnapshot) {
    motion = MotionDirection::Forward;
  } else if (countSnapshot < previousCountSnapshot) {
    motion = MotionDirection::Backward;
  } else if (stepDirection > 0) {
    motion = MotionDirection::Forward;
  } else if (stepDirection < 0) {
    motion = MotionDirection::Backward;
  }

  if (motion != MotionDirection::Idle) {
    lastMotionDirection = motion;
    lastMovementMillis  = millis();
  }

  previousCountSnapshot = countSnapshot;

  const int32_t forwardCount = countSnapshot < 0 ? 0 : countSnapshot;

  if (!pwIsLow && forwardCount >= TARGET_PULSES) {
    applyPwLow(F("✅ 5000 forward pulses reached → PW = LOW"));
  } else if (pwIsLow && motion == MotionDirection::Backward) {
    applyPwHigh(F("❌ Reverse rotation detected → PW = HIGH"));
  }

  reportStatus(countSnapshot, motion);
}

void updatePeState(float peVoltage) {
  if (peState == PeState::Active) {
    if (peVoltage >= PE_HIGH_THRESHOLD) {
      transitionToInactive();
    }
  } else if (peVoltage < PE_LOW_THRESHOLD) {
    transitionToActive();
  }
}

void IRAM_ATTR handleEncoderChange() {
  if (!countingEnabled) {
    return;
  }

  const uint8_t currentState = readEncoderPins();
  const uint8_t transitionIndex = static_cast<uint8_t>((lastEncoderState << 2) | currentState);

  // Gray-code transition table: +1 for forward steps, -1 for backward.
  static constexpr int8_t TRANSITION_TABLE[16] = {
      0, -1,  1,  0,
      1,  0,  0, -1,
     -1,  0,  0,  1,
      0,  1, -1,  0
  };

  const int8_t movement = TRANSITION_TABLE[transitionIndex];
  if (movement != 0) {
    encoderCount += movement;
    lastStepDirection = movement;
  }

  lastEncoderState = currentState;
}

void IRAM_ATTR onEncoderA() { handleEncoderChange(); }
void IRAM_ATTR onEncoderB() { handleEncoderChange(); }

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(PW_PIN, OUTPUT);

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266) || \
    defined(ARDUINO_ARCH_SAMD)  || defined(ARDUINO_ARCH_RP2040)
  analogReadResolution(ADC_RESOLUTION_BITS);
#endif

  applyPwHigh();
  lastEncoderState = readEncoderPins();

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), onEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), onEncoderB, CHANGE);

  Serial.println(F("⚙️ READY"));
}

void loop() {
  lastPeVoltage = readPeVoltage();
  updatePeState(lastPeVoltage);

  if (peState == PeState::Active) {
    managePwOutput();
  }

  delay(5);
}

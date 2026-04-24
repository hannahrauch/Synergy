#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// --- MOTOR PIN CONFIGURATION ---
// Using 5 PWM-capable pins on Arduino Uno: 3, 5, 6, 9, 10
// ⚠️ WARNING: Do NOT wire motors directly to these pins!
//    Each motor draws ~60-100mA; Arduino pins max out at 40mA per pin.
//    Use a transistor (e.g. 2N2222) or motor driver (e.g. L293D) per motor.
//    Wire: Arduino pin → transistor base (via 1kΩ resistor) → motor → 5V
const int motorPins[] = {3, 5, 6, 9, 10};
const int numMotors = 5; // Change to 6 if adding a 6th motor (use pin 11)

// --- DETECTION TUNING ---
// How much the smoothed acceleration delta must exceed to count as a tremor.
// Lower = more sensitive (picks up subtle tremors, might false-trigger on taps).
// Raise if the motor is triggering from normal hand movement.
float motionThreshold = 0.02;

// Motor PWM strength (0-255). 200 is a good starting point —
// strong enough to counter, not so strong it fights natural movement.
const int motorStrength = 200;

// --- SMOOTHING (EMA — Exponential Moving Average) ---
// Controls how quickly the filter responds to new sensor data.
// FIX: Was 0.3 (70% old data → heavy lag → motor fired AFTER tremor peaked).
// Now 0.7 means new data gets 70% weight → fast response → motor fires WITH tremor.
const float smoothing = 0.7;

float prevAccel = 0;
float smoothedDelta = 0;

// --- TIMING ---
// How long (ms) to keep motors ON after the last tremor spike is detected.
// Acts as a hold-off so the motor doesn't flicker on/off between tremor cycles.
// 150ms works well for 4-12Hz tremors (one full cycle is 83-250ms).
const int stopDelay = 150;

// Sample interval reduced from 50ms → 20ms (now 50Hz sampling)
// This gives us better resolution on fast tremors before smoothing is applied.
const int sampleInterval = 20;

unsigned long lastMotionTime = 0;
unsigned long lastSampleTime = 0;
bool motorOn = false;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();

  // Set all motor pins as outputs and make sure they start OFF
  for (int i = 0; i < numMotors; i++) {
    pinMode(motorPins[i], OUTPUT);
    analogWrite(motorPins[i], 0);
  }

  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050 not connected — check wiring!");
    while (1); // Halt if sensor isn't found
  }

  Serial.println("✅ MPU6050 ready. Monitoring for tremor...");
}

void loop() {
  unsigned long now = millis();

  // ─────────────────────────────────────────────
  // SENSOR BLOCK — runs every 20ms (non-blocking)
  // ─────────────────────────────────────────────
  if (now - lastSampleTime >= sampleInterval) {
    lastSampleTime = now;

    // Read raw accelerometer values from MPU6050
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    // Convert raw values to G-force (MPU6050 default range: ±2g → 16384 LSB/g)
    float x = ax / 16384.0;
    float y = ay / 16384.0;
    float z = az / 16384.0;

    // Compute total acceleration magnitude (removes orientation dependency)
    float accel = sqrt(x * x + y * y + z * z);

    // Delta = how much acceleration changed since last sample.
    // A tremor shows up as rapid, repeated changes in acceleration.
    float delta = abs(accel - prevAccel);
    prevAccel = accel;

    // Apply EMA smoothing to reduce noise while staying responsive.
    // FIX: smoothing raised from 0.3 → 0.7 so new data dominates.
    // Old behavior: smoothedDelta lagged 3-4 samples behind → motor fired late.
    // New behavior: smoothedDelta tracks the tremor in near real-time.
    smoothedDelta = (smoothing * delta) + ((1.0 - smoothing) * smoothedDelta);

    // Debug output — open Serial Monitor to watch this value.
    // When at rest it should hover near 0; a tremor will spike it above threshold.
    Serial.print("smoothedDelta: ");
    Serial.println(smoothedDelta, 4);

    // If we're above threshold, stamp the current time as "last tremor seen"
    if (smoothedDelta > motionThreshold) {
      lastMotionTime = now;
      Serial.println("  ↑ Tremor detected");
    }
  }

  // ─────────────────────────────────────────────────────────────
  // MOTOR CONTROL BLOCK — runs every loop iteration (non-blocking)
  //
  // Logic:  
  //   • If a tremor spike was seen within the last `stopDelay` ms → motors ON
  //   • If no tremor for longer than `stopDelay` ms              → motors OFF
  //
  // FIX: Because smoothing is now fast (0.7), lastMotionTime is updated
  // AT THE START of the tremor rather than after several lagged samples.
  // This means motors now turn on WITH the tremor, not after it.
  // ─────────────────────────────────────────────────────────────
  bool tremorActive = (millis() - lastMotionTime < stopDelay);

  if (tremorActive && !motorOn) {
    // Tremor just started (or resumed) — turn all motors ON
    for (int i = 0; i < numMotors; i++) {
      analogWrite(motorPins[i], motorStrength);
    }
    motorOn = true;
    Serial.println(">>> Motors ON");

  } else if (!tremorActive && motorOn) {
    // Tremor has stopped for longer than stopDelay — turn all motors OFF
    for (int i = 0; i < numMotors; i++) {
      analogWrite(motorPins[i], 0);
    }
    motorOn = false;
    Serial.println(">>> Motors OFF");
  }
}

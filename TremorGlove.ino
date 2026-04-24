#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
const int motorPin = 3;

float motionThreshold = 0.02;
float prevAccel = 0;
float smoothedDelta = 0;
const float smoothing = 0.3;

const int stopDelay = 150;
unsigned long lastMotionTime = 0;
unsigned long lastSampleTime = 0;
const int sampleInterval = 50; // ms between reads

bool motorOn = false;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();
  pinMode(motorPin, OUTPUT);

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected");
    while (1);
  }
  Serial.println("Ready");
}

void loop() {
  unsigned long now = millis();

  // --- SENSOR CHECK (non-blocking, runs every 50ms) ---
  if (now - lastSampleTime >= sampleInterval) {
    lastSampleTime = now;

    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    float x = ax / 16384.0;
    float y = ay / 16384.0;
    float z = az / 16384.0;

    float accel = sqrt(x * x + y * y + z * z);
    float delta = abs(accel - prevAccel);

    smoothedDelta = (smoothing * delta) + ((1.0 - smoothing) * smoothedDelta);
    prevAccel = accel;

    Serial.println(smoothedDelta);

    if (smoothedDelta > motionThreshold) {
      lastMotionTime = now;
    }
  }

  // --- MOTOR CONTROL (runs every loop iteration, no delay) ---
  if (millis() - lastMotionTime < stopDelay) {
    if (!motorOn) {
      analogWrite(motorPin, 255);
      motorOn = true;
    }
  } else {
    if (motorOn) {
      analogWrite(motorPin, 0);
      motorOn = false;
    }
  }
}
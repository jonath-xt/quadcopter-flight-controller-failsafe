#include <Wire.h>
#include <TinyGPSPlus.h>

const int motorPins[4] = {25, 26, 27, 14};  // FL, FR, RL, RR
const int throttlePin = 34;
const int battPin = 35;
const int armBtn = 33;
const int linkBtn = 32;   // held = signal lost
const int ledPin = 13;
const int trigPin = 5;
const int echoPin = 18;
const float fenceRadius = 20;   // metres

float Kp = 1.5, Ki = 0.5, Kd = 0.4;

enum State { DISARMED, ARMED, LANDING, CRASHED };
State state = DISARMED;
const char* names[] = {"DISARMED", "ARMED", "LANDING", "CRASHED"};

TinyGPSPlus gps;
double homeLat = 0, homeLng = 0;
bool homeSet = false;

float roll = 0, pitch = 0, rollI = 0, pitchI = 0;
float pitchSet = 0;
float landThrottle = 0;
unsigned long lastUs, lastBlink = 0;
bool ledOn = false;

int16_t rd() {
  int16_t h = Wire.read();
  return (h << 8) | Wire.read();
}

float readDistanceCm() {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long us = pulseIn(echoPin, HIGH, 12000);
  return us == 0 ? 400 : us * 0.0343 / 2;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);   // GPS
  Wire.begin(21, 22);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  for (int i = 0; i < 4; i++) ledcAttach(motorPins[i], 5000, 8);
  pinMode(armBtn, INPUT_PULLUP);
  pinMode(linkBtn, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  lastUs = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastUs) / 1e6;
  lastUs = now;

  // ---- feed GPS parser ----
  while (Serial2.available()) gps.encode(Serial2.read());

  // ---- read IMU ----
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  int16_t ax = rd(), ay = rd(), az = rd();
  rd();
  int16_t gx = rd(), gy = rd();
  rd();

  float accRoll  = atan2(ay, az) * 57.296;
  float accPitch = atan2(-ax, sqrt((float)ay * ay + (float)az * az)) * 57.296;
  roll  = 0.98 * (roll  + (gx / 131.0) * dt) + 0.02 * accRoll;
  pitch = 0.98 * (pitch + (gy / 131.0) * dt) + 0.02 * accPitch;

  // ---- inputs ----
  int throttle = map(analogRead(throttlePin), 0, 4095, 0, 255);
  float battV = 9.0 + analogRead(battPin) / 4095.0 * 3.6;
  bool linkLost = (digitalRead(linkBtn) == LOW);
  bool armPressed = (digitalRead(armBtn) == LOW);
  bool level = fabs(roll) < 10 && fabs(pitch) < 10;
  float dist = readDistanceCm();
  bool obstacle = dist < 100;

  // ---- failsafe state machine ----
  if ((state == ARMED || state == LANDING) && (fabs(roll) > 60 || fabs(pitch) > 60))
    state = CRASHED;

  switch (state) {
    case DISARMED:
      if (armPressed && throttle < 10 && level && !linkLost && battV > 10.5) {
        state = ARMED;
        rollI = 0; pitchI = 0;
        if (gps.location.isValid()) {          // record home point
          homeLat = gps.location.lat();
          homeLng = gps.location.lng();
          homeSet = true;
        } else homeSet = false;
      }
      break;
    case ARMED:
      if (linkLost || battV < 9.9) {
        state = LANDING;
        landThrottle = throttle;
      }
      break;
    case LANDING:
      landThrottle -= 40 * dt;
      if (landThrottle <= 0) { landThrottle = 0; state = DISARMED; }
      break;
    case CRASHED:
      if (throttle < 10 && level) state = DISARMED;
      break;
  }

  // ---- geofence ----
  float homeDist = 0, homeBearing = 0;
  bool breach = false;
  if (homeSet && gps.location.isValid()) {
    homeDist = TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(), homeLat, homeLng);
    homeBearing = TinyGPSPlus::courseTo(
      gps.location.lat(), gps.location.lng(), homeLat, homeLng);
    breach = (state == ARMED) && homeDist > fenceRadius;
  }

  // ---- obstacle brake (with integral reset when the setpoint changes) ----
  pitchSet = (obstacle && state == ARMED) ? 5 : 0;
  static float lastPitchSet = 0;
  if (pitchSet != lastPitchSet) { pitchI = 0; lastPitchSet = pitchSet; }

  // ---- PID + mixer ----
  int m[4] = {0, 0, 0, 0};
  if (state == ARMED || state == LANDING) {
    float thr = (state == ARMED) ? throttle : landThrottle;

    if (thr > 20) {
      rollI  = constrain(rollI  + roll * dt, -50, 50);
      pitchI = constrain(pitchI + (pitch - pitchSet) * dt, -50, 50);
    } else { rollI = 0; pitchI = 0; }

    float rollC  = Kp * roll + Ki * rollI + Kd * (gx / 131.0);
    float pitchC = Kp * (pitch - pitchSet) + Ki * pitchI + Kd * (gy / 131.0);

    m[0] = constrain((int)(thr - rollC - pitchC), 0, 255);
    m[1] = constrain((int)(thr + rollC - pitchC), 0, 255);
    m[2] = constrain((int)(thr - rollC + pitchC), 0, 255);
    m[3] = constrain((int)(thr + rollC + pitchC), 0, 255);
  } else {
    rollI = 0; pitchI = 0;
  }
  for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], m[i]);

  // ---- status LED ----
  int interval;
  if (state == DISARMED) interval = 500;
  else if (state == ARMED) interval = (battV < 10.5) ? 125 : (breach ? 300 : 0);
  else interval = 60;
  unsigned long ms = millis();
  if (interval == 0) { digitalWrite(ledPin, HIGH); }
  else if (ms - lastBlink >= (unsigned long)interval) {
    lastBlink = ms; ledOn = !ledOn; digitalWrite(ledPin, ledOn);
  }

  Serial.printf("%s | roll %.1f pitch %.1f | batt %.1fV | obst %.0fcm | home %.0fm | M %d %d %d %d\n",
    names[state], roll, pitch, battV, dist, homeDist, m[0], m[1], m[2], m[3]);
  if (breach)
    Serial.printf("!! FENCE BREACH - return home: %.0f m, bearing %.0f deg (%s)\n",
      homeDist, homeBearing, TinyGPSPlus::cardinal(homeBearing));
  delay(10);
}

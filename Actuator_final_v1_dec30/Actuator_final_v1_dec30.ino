// Core Arduino functions (Serial, pinMode, attachInterrupt, millis, etc.)
#include <Arduino.h>
// Cytron motor driver helper to control CytronMD motor objects
#include "CytronMotorDriver.h"

/* ================= CONFIG ================= */
// Serial baud rate for communication with host
#define BAUDRATE 115200
// Dead band for actuator target vs current pot reading (avoid chatter)
#define DEAD_BAND 20
// Main loop delta time in milliseconds (polling rate for actuators)
#define LOOP_DT 5   // ms (fast enough for ROS)

/* ================= MOTORS ================= */
// Motor driver objects: (mode, PWM pin, DIR pin)
CytronMD motor1(PWM_DIR, 3, 42);
CytronMD motor2(PWM_DIR, 2, 40);
CytronMD motor3(PWM_DIR, 5, 38);
CytronMD motor4(PWM_DIR, 4, 36);

/* ================= ENCODERS ================= */
// Encoder input pins
#define ENC1 18
#define ENC2 19
#define ENC3 20
#define ENC4 21

// Volatile counters incremented by ISRs on encoder events
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;
volatile long count4 = 0;

/* ================= ACTUATORS ================= */
// Actuator representation: pins for extend/retract, analog pot pin, pot min/max calibration,
// current target pot value, and whether the actuator is actively moving toward target.
struct Actuator {
  int extendPin;
  int retractPin;
  int potPin;
  int potMin;
  int potMax;
  int target;
  bool active;
};
  // extend, retract, pot,  min, max, target, active

// Array of three actuators with pin mappings and calibration values
Actuator actuators[3] = {
  {2, 3, A0,  460, 700, 460, false},
  {4, 5, A1,  150, 500, 150, false},
  {6, 7, A2,  85, 470,  85, false}
};

/* ================= ISR ================= */
// Simple ISRs: increment encoder counters on each triggered edge
void isr1(){ count1++; }
void isr2(){ count2++; }
void isr3(){ count3++; }
void isr4(){ count4++; }

/* ================= HELPERS ================= */
// Read actuator potentiometer and clamp to calibrated min/max range
inline int readPot(Actuator &a) {
  return constrain(analogRead(a.potPin), a.potMin, a.potMax);
}

// Stop movement of a single actuator by disabling both drive pins
inline void stopActuator(Actuator &a) {
  digitalWrite(a.extendPin, LOW);
  digitalWrite(a.retractPin, LOW);
}

// Drive actuator to extend: set extend pin HIGH, retract LOW
inline void extendActuator(Actuator &a) {
  digitalWrite(a.extendPin, HIGH);
  digitalWrite(a.retractPin, LOW);
}

// Drive actuator to retract: set retract pin HIGH, extend LOW
inline void retractActuator(Actuator &a) {
  digitalWrite(a.extendPin, LOW);
  digitalWrite(a.retractPin, HIGH);
}

// Set motor speeds for left and right sides (clamp inputs and apply to both motors per side)
inline void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  motor1.setSpeed(left);
  motor2.setSpeed(left);
  motor3.setSpeed(right);
  motor4.setSpeed(right);
}

/* ================= SERIAL ================= */
// Simple serial command interface. Single-character commands followed by numeric args where applicable.
// Commands supported:
//  'm' <left> <right>    -> set motor speeds
//  's' <t0> <t1> <t2>    -> set actuator targets (pot values)
//  'e'                   -> request encoder counts (returns "<left> <right>\n")
//  'a'                   -> request actuator pot readings (returns "p0 p1 p2\n")
void handleSerial()
{
  if (!Serial.available()) return;

  char cmd = Serial.read();

  /* MOTOR COMMAND */
  if (cmd == 'm') {
    int l = Serial.parseInt();
    int r = Serial.parseInt();
    setMotors(l, r);
  }

  // Set actuator target positions for all 3 actuators
  else if (cmd == 's') {
    for (int i = 0; i < 3; i++) {
      int t = Serial.parseInt();
      // clamp incoming target to calibrated pot range
      t = constrain(t, actuators[i].potMin, actuators[i].potMax);

      int current = readPot(actuators[i]);

      // Only activate movement if the requested target differs by more than DEAD_BAND
      if (abs(t - current) > DEAD_BAND) {
        actuators[i].target = t;
        actuators[i].active = true;
      }
    }
}

  /* ENCODERS (ALWAYS RESPOND) */
  else if (cmd == 'e') {
    long left  = (count1 + count2) / 2;
    long right = (count3 + count4) / 2;
    Serial.print(left);
    Serial.print(" ");
    Serial.print(right);
    Serial.print("\n");
  }

  /* ACTUATORS FEEDBACK (ALWAYS RESPOND) */
  else if (cmd == 'a') {
    // Print current potentiometer readings for each actuator
    Serial.print(readPot(actuators[0]));
    Serial.print(" ");
    Serial.print(readPot(actuators[1]));
    Serial.print(" ");
    Serial.print(readPot(actuators[2]));
    Serial.print("\n");
  }

  // Clear any remaining bytes in the Serial buffer for this command cycle
  while (Serial.available()) Serial.read();
}

/* ================= ACTUATOR LOOP ================= */
// Called repeatedly to drive an active actuator toward its target pot reading.
void updateActuator(Actuator &a)
{
  if (!a.active) return;

  int pos = readPot(a);
  int err = a.target - pos;

  // If within dead band, stop and mark inactive
  if (abs(err) <= DEAD_BAND) {
    stopActuator(a);
    a.active = false;
    return;
  }

  // Drive in the correct direction based on sign of error
  if (err > 0) extendActuator(a);
  else retractActuator(a);
}

/* ================= SETUP ================= */
void setup()
{
  // Initialize serial for host communication
  Serial.begin(BAUDRATE);

  // Configure encoder pins with internal pull-ups
  pinMode(ENC1, INPUT_PULLUP);
  pinMode(ENC2, INPUT_PULLUP);
  pinMode(ENC3, INPUT_PULLUP);
  pinMode(ENC4, INPUT_PULLUP);

  // Attach ISRs for encoder counting on FALLING edges
  attachInterrupt(digitalPinToInterrupt(ENC1), isr1, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC2), isr2, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC3), isr3, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC4), isr4, FALLING);

  // Configure actuator control pins and ensure actuators are stopped
  for (int i = 0; i < 3; i++) {
    pinMode(actuators[i].extendPin, OUTPUT);
    pinMode(actuators[i].retractPin, OUTPUT);
    stopActuator(actuators[i]);
  }

  // Signal ready to host
  Serial.println("ZENORAK READY");
}

/* ================= LOOP ================= */
void loop()
{
  // Always process serial commands first for responsiveness
  handleSerial();   // ALWAYS FIRST

  // Simple fixed-rate loop using millis() and LOOP_DT to schedule actuator updates
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < LOOP_DT) return;
  last = now;

  // Update each actuator (move toward target if active)
  for (int i = 0; i < 3; i++)
    updateActuator(actuators[i]);
}
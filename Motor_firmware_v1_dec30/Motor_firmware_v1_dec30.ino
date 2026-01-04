// Include core Arduino API (digital/analog, Serial, pinMode, attachInterrupt, etc.)
#include <Arduino.h>
// Include Cytron motor driver helper to control CytronMD objects
#include "CytronMotorDriver.h"

/* ================= CONFIG ================= */
// Define serial baud rate used for Serial communication with the host
#define BAUDRATE 115200

/* ================= MOTORS ================= */
// Using CytronMD in PWM_DIR mode: first arg selects mode, next two are PWM and DIR pins
// Front-left motor object (PWM pin 3, DIR pin 42)
CytronMD motorFL(PWM_DIR, 3, 42);  // Front Left
// Rear-left motor object (PWM pin 2, DIR pin 40)
CytronMD motorRL(PWM_DIR, 2, 40);  // Rear Left
// Front-right motor object (PWM pin 5, DIR pin 38)
CytronMD motorFR(PWM_DIR, 5, 38);  // Front Right
// Rear-right motor object (PWM pin 4, DIR pin 36)
CytronMD motorRR(PWM_DIR, 4, 36);  // Rear Right

/* ================= ENCODERS ================= */
// Encoder input pin definitions (one pin per encoder channel used here)
#define ENC_FL 18
#define ENC_RL 19
#define ENC_FR 20
#define ENC_RR 21

// Volatile counters for encoder ticks; volatile because updated inside ISRs
volatile long encFL = 0;
volatile long encRL = 0;
volatile long encFR = 0;
volatile long encRR = 0;

/* ================= ISR ================= */
// Interrupt Service Routines: each is called on the encoder signal edge and increments its counter
void isrFL() { encFL++; }
void isrRL() { encRL++; }
void isrFR() { encFR++; }
void isrRR() { encRR++; }

/* ================= MOTOR CONTROL ================= */
// Helper to set left/right motor speeds. Values are clamped to valid PWM range.
inline void setMotors(int left, int right)
{
  // Clamp inputs to [-255, 255] where the Cytron driver expects signed speed values
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  // Apply the left speed to both left-side motors
  motorFL.setSpeed(left);
  motorRL.setSpeed(left);
  // Apply the right speed to both right-side motors
  motorFR.setSpeed(right);
  motorRR.setSpeed(right);
}

/* ================= SERIAL PARSER ================= */
// Fast, non-blocking, CR-terminated command parser.
// Supported commands:
//   m<left> <right>\r   -> set motor speeds (integers)
//   e\r                 -> request encoder counts (returns "<left> <right>\n")
char rxBuf[32];          // buffer for incoming command characters
uint8_t rxIndex = 0;     // current write index into rxBuf

void handleSerial()
{
  // Read all available characters from Serial (non-blocking)
  while (Serial.available())
  {
    char c = Serial.read();

    // If we get a carriage-return, treat it as end-of-command
    if (c == '\r')
    {
      // Null-terminate received string and reset index for next command
      rxBuf[rxIndex] = '\0';
      rxIndex = 0;

      // ---- MOTOR COMMAND (starts with 'm') ----
      if (rxBuf[0] == 'm')
      {
        int l = 0, r = 0;
        // Parse two integers after the 'm'
        sscanf(rxBuf + 1, "%d %d", &l, &r);
        // Set motor speeds (left, right)
        setMotors(l, r);
      }

      // ---- ENCODER REQUEST (starts with 'e') ----
      else if (rxBuf[0] == 'e')
      {
        // Disable interrupts briefly to read multi-byte volatile counters safely
        noInterrupts();
        long left  = (encFL + encRL) / 2;   // average left side encoders
        long right = (encFR + encRR) / 2;   // average right side encoders
        interrupts();

        // Send encoder counts as text: "<left> <right>\n"
        Serial.print(left);
        Serial.print(' ');
        Serial.print(right);
        Serial.print('\n');
      }
    }
    else
    {
      // Accumulate character into buffer if there is space (leave room for NUL)
      if (rxIndex < sizeof(rxBuf) - 1)
        rxBuf[rxIndex++] = c;
    }
  }
}

/* ================= SETUP ================= */
void setup()
{
  // Initialize serial at configured baud rate for communication with host
  Serial.begin(BAUDRATE);

  // Configure encoder pins as inputs with internal pull-ups enabled
  pinMode(ENC_FL, INPUT_PULLUP);
  pinMode(ENC_RL, INPUT_PULLUP);
  pinMode(ENC_FR, INPUT_PULLUP);
  pinMode(ENC_RR, INPUT_PULLUP);

  // Attach interrupt handlers for each encoder pin on the FALLING edge
  attachInterrupt(digitalPinToInterrupt(ENC_FL), isrFL, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_RL), isrRL, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR), isrFR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR), isrRR, FALLING);

  // Ensure motors are stopped at startup
  setMotors(0, 0);

  // Notify host that the motor controller is ready
  Serial.println("ZENORAK MOTOR READY");
}

/* ================= LOOP ================= */
void loop()
{
  // No delay here: repeatedly call handleSerial() to be responsive to commands
  handleSerial();
}
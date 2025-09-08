#include <Arduino.h>
#include "CytronMotorDriver.h"

// Create Cytron Objects (PWM_DIR, PWM, DIR)
CytronMD motor1(PWM_DIR, 3, 2);
CytronMD motor2(PWM_DIR, 5, 4);
CytronMD motor3(PWM_DIR, 6, 7);
CytronMD motor4(PWM_DIR, 9, 8);

#define CMD_BUFFER_SIZE 40
char inputBuffer[CMD_BUFFER_SIZE];
byte bufferIndex = 0;

// ====== Variables ======
char lastAction = 's';
int lastSpeed = 0;

unsigned long lastCommandTime = 0;     // watchdog
const unsigned long COMMAND_TIMEOUT = 1000; // 1 second

// ====== Motor Control Functions ======
void forward(int speed) {
  motor1.setSpeed(-speed);
  motor2.setSpeed(-speed);
  motor3.setSpeed(-speed);
  motor4.setSpeed(speed);
}

void backward(int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(-speed);
}

void left(int speed) {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(-speed);
  motor4.setSpeed(speed);
}

void right(int speed) {
  motor1.setSpeed(-speed);
  motor2.setSpeed(-speed);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}

void spinLeft(int speed) {
  motor1.setSpeed(-speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(-speed);
  motor4.setSpeed(speed);
}

void spinRight(int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(-speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(-speed);
}

void stopMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}

// ====== Command Processor ======
void processCommand(const char *cmd) {
  if (strlen(cmd) < 2) return;

  char action = cmd[0];
  int speed = atoi(cmd + 1);
  speed = constrain(speed, 0, 255);

  // Skip if same as last command
  if (action == lastAction && speed == lastSpeed) return;

  switch (action) {
    case 'f': forward(speed); break;
    case 'b': backward(speed); break;
    case 'l': left(speed); break;
    case 'r': right(speed); break;
    case 'e': spinLeft(speed); break;
    case 'q': spinRight(speed); break;
    case 's': stopMotors(); break;
    default: return; // ignore unknown
  }

  lastAction = action;
  lastSpeed = speed;
  lastCommandTime = millis(); // reset watchdog

  Serial.print("Executed: ");
  Serial.println(cmd);
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  stopMotors();
  Serial.println("Motor Control Ready (Non-blocking + Watchdog)!");
  lastCommandTime = millis();
}

// ====== Loop ======
void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      inputBuffer[bufferIndex] = '\0';  // null-terminate string
      processCommand(inputBuffer);
      bufferIndex = 0;                  // reset buffer
    } 
    else if (c != '\r') {
      if (bufferIndex < CMD_BUFFER_SIZE - 1) {
        inputBuffer[bufferIndex++] = c;
      }
    }
  }

  // ===== Watchdog check =====
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    stopMotors();
    lastAction = 's';
    lastSpeed = 0;
  }
}

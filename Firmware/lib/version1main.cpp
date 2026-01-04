/******************************************************************************
 * Filename: firmware_arduino.cpp
 * Description: Arduino firmware to control a 4-motor robot using the Cytron 
 *              motor driver. Commands are received via Serial and executed 
 *              in a non-blocking way.
 * Author: Your Name
 * Date: YYYY-MM-DD
 ******************************************************************************/

#include <Arduino.h>
#include "CytronMotorDriver.h" // Include the library for Cytron motor drivers

// ==================== Motor Objects ====================
// Create four CytronMD objects for controlling 4 motors in PWM_DIR mode.
// PWM_DIR mode: one pin for PWM speed control, one pin for direction.
CytronMD motor1(PWM_DIR, 3, 2);  // Motor 1 on PWM=3, DIR=2
CytronMD motor2(PWM_DIR, 5, 4);  // Motor 2 on PWM=5, DIR=4
CytronMD motor3(PWM_DIR, 6, 7);  // Motor 3 on PWM=6, DIR=7
CytronMD motor4(PWM_DIR, 9, 8);  // Motor 4 on PWM=9, DIR=8

// ==================== Serial Command Buffer ====================
// We will read commands sent over Serial into a char buffer
#define CMD_BUFFER_SIZE 40          // Maximum command length
char inputBuffer[CMD_BUFFER_SIZE];  // Stores incoming command characters
byte bufferIndex = 0;               // Tracks position in buffer

// ==================== Variables ====================
char lastAction = 's';  // Stores last motor action (f/b/l/r/e/q/s)
int lastSpeed = 0;      // Stores last speed value

// ==================== Motor Control Functions ====================
// Each function controls the 4 motors according to the desired motion.
// speed ranges from 0 (stop) to 255 (full speed). Negative speeds reverse motor direction.

void forward(int speed) {
  motor1.setSpeed(-speed); // Reverse because of motor orientation
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
  // Reduce power on two motors for smoother turning
  motor1.setSpeed((-speed)/4);
  motor2.setSpeed((-speed)/4);
  motor3.setSpeed(-speed);
  motor4.setSpeed(speed);
}

void right(int speed) {
  // Reduce power on two motors for smoother turning
  motor1.setSpeed(-speed);
  motor2.setSpeed(-speed);
  motor3.setSpeed((-speed)/4);
  motor4.setSpeed((speed)/4);
}

void spinLeft(int speed) {
  // Spin robot in place to left by setting opposite motor directions
  motor1.setSpeed(-speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(-speed);
  motor4.setSpeed(speed);
}

void spinRight(int speed) {
  // Spin robot in place to right
  motor1.setSpeed(speed);
  motor2.setSpeed(-speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(-speed);
}

void stopMotors() {
  // Immediately stop all motors
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}

// ==================== Command Processor ====================
// This function processes a command received from Serial.
// Command format: <action><speed> e.g., "f120" -> move forward at speed 120
void processCommand(const char *cmd) {
  if (strlen(cmd) < 2) return;  // Ignore invalid or empty commands

  char action = cmd[0];          // First character determines action
  int speed = atoi(cmd + 1);     // Convert remaining characters to integer speed
  speed = constrain(speed, 0, 255); // Ensure speed is within valid range

  // Skip repeated commands (optimization)
  if (action == lastAction && speed == lastSpeed) return;

  // Execute the motor command based on action
  switch (action) {
    case 'f': forward(speed); break;
    case 'b': backward(speed); break;
    case 'l': left(speed); break;
    case 'r': right(speed); break;
    case 'e': spinLeft(speed); break;
    case 'q': spinRight(speed); break;
    case 's': stopMotors(); break;
    default: return; // Ignore unknown commands
  }

  // Update last command values for next iteration
  lastAction = action;
  lastSpeed = speed;

  // Print to Serial for debugging
  Serial.print("Executed: ");
  Serial.println(cmd);
}

// ==================== Setup ====================
// Runs once when Arduino starts
void setup() {
  Serial.begin(115200);  // Initialize Serial communication at 115200 baud
  stopMotors();          // Ensure all motors are stopped initially
  Serial.println("Motor Control Ready (Non-blocking)!");
}

// ==================== Loop ====================
// Continuously checks for Serial input and executes commands
void loop() {
  while (Serial.available() > 0) { // While there is Serial data
    char c = Serial.read();        // Read one character

    if (c == '\n') {               // Command terminated by newline
      inputBuffer[bufferIndex] = '\0'; // Null-terminate buffer
      processCommand(inputBuffer);      // Execute the command
      bufferIndex = 0;                  // Reset buffer index
    } 
    else if (c != '\r') {           // Ignore carriage return
      if (bufferIndex < CMD_BUFFER_SIZE - 1) { // Prevent buffer overflow
        inputBuffer[bufferIndex++] = c;       // Add character to buffer
      }
    }
  }
}

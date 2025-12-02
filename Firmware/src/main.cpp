#include <Arduino.h>
#include "CytronMotorDriver.h"

// Create Cytron Motor Objects (PWM_DIR, PWM, DIR)
CytronMD motor1(PWM_DIR, 3,  2); // Left motor 1
CytronMD motor2(PWM_DIR, 5, 4); // Left motor 2
CytronMD motor3(PWM_DIR, 6, 7); // Right motor 1
CytronMD motor4(PWM_DIR, 9, 8); // Right motor 2

void setup() {
  Serial.begin(57600);  // Baud rate for serial communication
  Serial.println("Motor Control Ready!");
}

void forward(int leftSpeed, int rightSpeed) {
  motor1.setSpeed(leftSpeed);
  motor2.setSpeed(leftSpeed);
  motor3.setSpeed(rightSpeed);
  motor4.setSpeed(rightSpeed);
}

void backward(int leftSpeed, int rightSpeed) {
  motor1.setSpeed(-leftSpeed);
  motor2.setSpeed(-leftSpeed);
  motor3.setSpeed(-rightSpeed);
  motor4.setSpeed(-rightSpeed);
}

void stopMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}

void loop() {
  static String input = "";

  // Read characters as they arrive
  while (Serial.available()) {
    char ch = Serial.read();

    // If newline or carriage return received, process the command
    if (ch == '\n' || ch == '\r') {
      input.trim();  // Remove leading/trailing spaces

      if (input.length() > 0) {
        // Parse the first character for command type ('m' or 'e')
        char commandType = input.charAt(0);

        // Remove the first character and trim any spaces
        input = input.substring(1);
        input.trim();

        int leftSpeed = 0, rightSpeed = 0;
        int parsed = sscanf(input.c_str(), "%d %d", &leftSpeed, &rightSpeed);

        // Check if two speeds were parsed successfully
        if (parsed == 2) {
          // Constrain speeds to the valid range (-255 to 255)
          leftSpeed = constrain(leftSpeed, -255, 255);
          rightSpeed = constrain(rightSpeed, -255, 255);

          // Execute the appropriate action based on the command type
          switch (commandType) {
            case 'm':  // Move forward/backward
              forward(leftSpeed, rightSpeed);
              break;
            case 'e':  // Spin left/right
              if (leftSpeed > 0 && rightSpeed > 0) {
                forward(leftSpeed, rightSpeed); // Forward movement
              } else if (leftSpeed < 0 && rightSpeed < 0) {
                backward(leftSpeed, rightSpeed); // Backward movement
              }
              break;
            default:
              stopMotors();
              break;
          }

          // Send back the parsed values via Serial for debugging
          Serial.print("Parsed Successfully! ");
          Serial.print("Left speed (l): ");
          Serial.print(leftSpeed);
          Serial.print(" Right speed (r): ");
          Serial.println(rightSpeed);
        } else {
          // If parsing fails, send an error message
          Serial.println("Invalid command format! Please provide two speed values.");
        }
      }

      input = "";  // Clear input buffer for the next command
    } else {
      input += ch;  // Build the input string character by character
    }
  }
}

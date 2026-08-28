/*
  Motor minimum-PWM threshold test

  Commands in Serial Monitor:
    1 = MTR1 forward
    2 = MTR1 reverse
    3 = MTR2 forward
    4 = MTR2 reverse
    u = increase PWM
    d = decrease PWM
    x = stop
    h = show help

  Serial Monitor baud: 115200
*/

#include <Arduino.h>

// Onboard motor-driver control pins
const int M1H = 45;
const int M1L = 46;
const int M2H = 43;
const int M2L = 44;

const int PWM_FREQUENCY = 12000;
const int PWM_RESOLUTION = 8;
const int PWM_STEP = 5;

enum TestSelection {
  NONE,
  MTR1_FORWARD,
  MTR1_REVERSE,
  MTR2_FORWARD,
  MTR2_REVERSE
};

TestSelection selectedTest = NONE;
int testPwm = 0;

void stopAllMotors() {
  ledcWrite(M1H, 0);
  ledcWrite(M1L, 0);
  ledcWrite(M2H, 0);
  ledcWrite(M2L, 0);
}

void applyTestOutput() {
  stopAllMotors();

  switch (selectedTest) {
    case MTR1_FORWARD:
      ledcWrite(M1H, testPwm+150);
      break;

    case MTR1_REVERSE:
      ledcWrite(M1L, testPwm+150);
      break;

    case MTR2_FORWARD:
      ledcWrite(M2H, testPwm+150);
      break;

    case MTR2_REVERSE:
      ledcWrite(M2L, testPwm+150);
      break;

    case NONE:
      break;
  }
}

const char* testName() {
  switch (selectedTest) {
    case MTR1_FORWARD: return "MTR1 forward";
    case MTR1_REVERSE: return "MTR1 reverse";
    case MTR2_FORWARD: return "MTR2 forward";
    case MTR2_REVERSE: return "MTR2 reverse";
    default: return "none";
  }
}

void printStatus() {
  Serial.print("Test: ");
  Serial.print(testName());
  Serial.print(" | PWM: ");
  Serial.println(testPwm);
}

void selectTest(TestSelection newTest) {
  stopAllMotors();
  selectedTest = newTest;
  testPwm = 0;
  applyTestOutput();
  printStatus();
}

void printHelp() {
  Serial.println();
  Serial.println("Motor threshold test");
  Serial.println("1: MTR1 forward");
  Serial.println("2: MTR1 reverse");
  Serial.println("3: MTR2 forward");
  Serial.println("4: MTR2 reverse");
  Serial.println("u: increase PWM by 5");
  Serial.println("d: decrease PWM by 5");
  Serial.println("x: stop");
  Serial.println("h: help");
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  ledcAttach(M1H, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(M1L, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(M2H, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(M2L, PWM_FREQUENCY, PWM_RESOLUTION);

  stopAllMotors();

  delay(1000);
  printHelp();
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  char command = Serial.read();

  switch (command) {
    case '1':
      selectTest(MTR1_FORWARD);
      break;

    case '2':
      selectTest(MTR1_REVERSE);
      break;

    case '3':
      selectTest(MTR2_FORWARD);
      break;

    case '4':
      selectTest(MTR2_REVERSE);
      break;

    case 'u':
    case 'U':
      if (selectedTest != NONE) {
        testPwm = min(testPwm + PWM_STEP, 255);
        applyTestOutput();
        printStatus();
      }
      break;

    case 'd':
    case 'D':
      if (selectedTest != NONE) {
        testPwm = max(testPwm - PWM_STEP, 0);
        applyTestOutput();
        printStatus();
      }
      break;

    case 'x':
    case 'X':
      selectedTest = NONE;
      testPwm = 0;
      stopAllMotors();
      printStatus();
      break;

    case 'h':
    case 'H':
      printHelp();
      break;
  }
}

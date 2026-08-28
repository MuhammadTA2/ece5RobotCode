/* *************************************************************************************************
   UCSD ECE 5 Lab 4: Simplified PID Line-Following Robot

   LF_original.ino remains the untouched professor-provided reference. This sketch keeps the
   original control structure while making the speed potentiometer useful across its full travel.

   SPEED POTENTIOMETER BEHAVIOR
   - Fully down: both motors stop.
   - Above the small stop dead zone: output begins at the measured usable motor PWM of 170.
   - Fully up: output reaches PWM 255.
   - Intermediate positions are proportional across the usable 170-255 motor range.

   The jump from 0 to 170 is intentional: testing showed that the motors do not rotate reliably
   below PWM 170. Change the documented constants below if later hardware tests give new limits.
************************************************************************************************* */

#include <Arduino.h>
#include <math.h>

// =================================================================================================
// ADJUSTABLE SETTINGS

// Turn off for untethered runs after tuning.
const bool ENABLE_SERIAL_TELEMETRY = true;
const unsigned long SERIAL_BAUD_RATE = 115200;
const unsigned long TELEMETRY_INTERVAL_MS = 50;

// A fixed sample interval makes the original sample-based I and D terms repeatable.
const unsigned long CONTROL_INTERVAL_MS = 10;

// Sensor calibration and detection.
const int CALIBRATION_MEASUREMENTS = 20;
const unsigned long CALIBRATION_COLOR_CHANGE_MS = 2000;
// The original accepts even small white/black spans. Keep only a small guard against division by
// zero; raise this later only if telemetry proves that a sensor is dominated by noise.
const float MIN_CALIBRATION_SPAN = 1.0f;

// The original stores 1.5 in an integer CriteriaForMax, so its effective comparison ratio is 1.0.
// Matching that behavior prevents valid side-sensor readings from being rejected before PID runs.
const float PEAK_TO_AVERAGE_RATIO = 1.00f;

// Speed-pot calibration. If the knob works backward, change SPEED_POT_REVERSED to true. If the
// physical potentiometer does not reach 0 or 4095, replace the endpoint values with measured ADCs.
const int SPEED_POT_ADC_MIN = 0;
const int SPEED_POT_ADC_MAX = 4095;
const int SPEED_POT_STOP_DEAD_ZONE_ADC = 80;
const bool SPEED_POT_REVERSED = false;

// Measured usable motor range. PWM 170 was the minimum in all four tested motor directions.
const int MIN_EFFECTIVE_MOTOR_PWM = 170;
const int MAX_MOTOR_PWM = 255;

// PID potentiometers are read as 0-100, then multiplied by these original-style scale factors.
// Tune P first with I and D at zero. Add D for damping and only a small I for persistent bias.
const float KP_SCALE = 1.0f;
const float KI_SCALE = 0.001f;
const float KD_SCALE = 0.01f;
const float INTEGRAL_SUM_LIMIT = 5.0f;
const float CENTERED_ERROR_BAND = 0.05f;

// Both wheels remain forward. This is the largest useful correction between the measured minimum
// and maximum PWM values. Lower it if steering is too abrupt.
const int MAX_TURN_PWM = MAX_MOTOR_PWM - MIN_EFFECTIVE_MOTOR_PWM;

// Positive sensor error means the line is on the right. Change to -1 only if wiring reverses turns.
const float STEERING_DIRECTION = 1.0f;

// =================================================================================================
// HARDWARE PINS

const int LDR_PINS[] = {1, 2, 3, 4, 5, 6, 7};  // Left-to-right when viewed from behind the rover.
const int SENSOR_COUNT = sizeof(LDR_PINS) / sizeof(LDR_PINS[0]);

const int M1H = 45;
const int M1L = 46;
const int M2H = 43;
const int M2L = 44;

const int SPEED_POT_PIN = 10;
const int P_POT_PIN = 11;
const int I_POT_PIN = 12;
const int D_POT_PIN = 13;

const int LED_PINS[] = {17};
const int LED_COUNT = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

const int MOTOR_PWM_FREQUENCY = 12000;
const int MOTOR_PWM_RESOLUTION_BITS = 8;

enum BoardMotor { MOTOR_1, MOTOR_2 };

// =================================================================================================
// RUNTIME VALUES

float calibrationWhite[SENSOR_COUNT] = {0};
float calibrationBlack[SENSOR_COUNT] = {0};
int rawLdr[SENSOR_COUNT] = {0};
int normalizedLdr[SENSOR_COUNT] = {0};

int rawSpeedPot = 0;
int rawPPot = 0;
int rawIPot = 0;
int rawDPot = 0;
int baseMotorPwm = 0;

float kP = 0.0f;
float kI = 0.0f;
float kD = 0.0f;

float lineError = 0.0f;
float previousError = 0.0f;
float integralError = 0.0f;
float pTerm = 0.0f;
float iTerm = 0.0f;
float dTerm = 0.0f;
float rawTurn = 0.0f;
int turnPwm = 0;
bool lineDetected = false;

int leftMotorPwm = 0;
int rightMotorPwm = 0;

unsigned long lastControlMs = 0;
unsigned long lastTelemetryMs = 0;

// =================================================================================================
// GENERAL HELPERS

int clampInt(int value, int minimum, int maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

void setLeds(int value) {
  for (int led = 0; led < LED_COUNT; led++) digitalWrite(LED_PINS[led], value);
}

// =================================================================================================
// CALIBRATION AND SENSOR INPUT

void calibrateColor(bool calibratingBlack) {
  Serial.println(calibratingBlack ? "\nCalibrating black" : "\nCalibrating white");

  for (int blink = 0; blink < 4; blink++) {
    setLeds(1);
    delay(250);
    setLeds(0);
    delay(250);
  }

  setLeds(1);
  float sums[SENSOR_COUNT] = {0};
  for (int measurement = 0; measurement < CALIBRATION_MEASUREMENTS; measurement++) {
    for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
      sums[sensor] += (float)analogRead(LDR_PINS[sensor]);
      delay(2);
    }
  }

  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    float average = sums[sensor] / (float)CALIBRATION_MEASUREMENTS;
    if (calibratingBlack) calibrationBlack[sensor] = average;
    else calibrationWhite[sensor] = average;
  }
}

void printCalibrationValues() {
  Serial.print("White: ");
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    Serial.print(calibrationWhite[sensor], 0);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("Black: ");
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    Serial.print(calibrationBlack[sensor], 0);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("Absolute spans: ");
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    Serial.print(fabsf(calibrationBlack[sensor] - calibrationWhite[sensor]), 0);
    Serial.print(' ');
  }
  Serial.println();
}

void calibrateSensors() {
  calibrateColor(false);  // White first.
  setLeds(0);
  delay(CALIBRATION_COLOR_CHANGE_MS);
  calibrateColor(true);   // Black second.
  printCalibrationValues();
  setLeds(1);
  delay(CALIBRATION_COLOR_CHANGE_MS);
}

void readPhotoresistors() {
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    rawLdr[sensor] = analogRead(LDR_PINS[sensor]);
    float span = calibrationBlack[sensor] - calibrationWhite[sensor];

    if (fabsf(span) < MIN_CALIBRATION_SPAN) {
      normalizedLdr[sensor] = 0;
      continue;
    }

    float normalized = 100.0f * ((float)rawLdr[sensor] - calibrationWhite[sensor]) / span;
    normalizedLdr[sensor] = (int)roundf(clampFloat(normalized, 0.0f, 100.0f));
  }
}

bool calculateLineError() {
  int peakIndex = 0;
  int peakValue = -1;
  float average = 0.0f;

  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    int value = normalizedLdr[sensor];
    average += (float)value / (float)SENSOR_COUNT;
    if (value > peakValue) {
      peakValue = value;
      peakIndex = sensor;
    }
  }

  if ((float)peakValue <= PEAK_TO_AVERAGE_RATIO * average) {
    return false;
  }

  int leftNeighbor = max(0, peakIndex - 1);
  int rightNeighbor = min(SENSOR_COUNT - 1, peakIndex + 1);
  float denominator = (float)normalizedLdr[leftNeighbor]
                    + (float)normalizedLdr[peakIndex]
                    + (float)normalizedLdr[rightNeighbor];
  if (denominator <= 0.0f) return false;

  float numerator = (float)normalizedLdr[leftNeighbor] * (float)leftNeighbor
                  + (float)normalizedLdr[peakIndex] * (float)peakIndex
                  + (float)normalizedLdr[rightNeighbor] * (float)rightNeighbor;
  float weightedPosition = numerator / denominator;
  lineError = weightedPosition - ((float)(SENSOR_COUNT - 1) / 2.0f);
  return true;
}

// =================================================================================================
// POTENTIOMETERS

int mapSpeedPotToPwm(int rawAdc) {
  int boundedAdc = clampInt(rawAdc, SPEED_POT_ADC_MIN, SPEED_POT_ADC_MAX);
  int orientedAdc = boundedAdc;
  if (SPEED_POT_REVERSED) {
    orientedAdc = SPEED_POT_ADC_MAX - (boundedAdc - SPEED_POT_ADC_MIN);
  }

  int activeStartAdc = SPEED_POT_ADC_MIN + SPEED_POT_STOP_DEAD_ZONE_ADC;
  if (orientedAdc <= activeStartAdc) return 0;

  float fraction = (float)(orientedAdc - activeStartAdc)
                 / (float)(SPEED_POT_ADC_MAX - activeStartAdc);
  return (int)roundf(
    (float)MIN_EFFECTIVE_MOTOR_PWM
    + clampFloat(fraction, 0.0f, 1.0f)
    * (float)(MAX_MOTOR_PWM - MIN_EFFECTIVE_MOTOR_PWM)
  );
}

float potAdcToHundred(int rawAdc) {
  return 100.0f * (float)clampInt(rawAdc, 0, 4095) / 4095.0f;
}

void readPotentiometers() {
  rawSpeedPot = analogRead(SPEED_POT_PIN);
  rawPPot = analogRead(P_POT_PIN);
  rawIPot = analogRead(I_POT_PIN);
  rawDPot = analogRead(D_POT_PIN);

  baseMotorPwm = mapSpeedPotToPwm(rawSpeedPot);
  kP = potAdcToHundred(rawPPot) * KP_SCALE;
  kI = potAdcToHundred(rawIPot) * KI_SCALE;
  kD = potAdcToHundred(rawDPot) * KD_SCALE;
}

// =================================================================================================
// PID CONTROL

void resetPid() {
  previousError = lineError;
  integralError = 0.0f;
  pTerm = 0.0f;
  iTerm = 0.0f;
  dTerm = 0.0f;
  rawTurn = 0.0f;
  turnPwm = 0;
}

void calculatePidTurn() {
  integralError = clampFloat(
    integralError + lineError,
    -INTEGRAL_SUM_LIMIT,
    INTEGRAL_SUM_LIMIT
  );
  if (fabsf(lineError) <= CENTERED_ERROR_BAND) integralError = 0.0f;

  pTerm = lineError * kP;
  iTerm = integralError * kI;
  dTerm = (lineError - previousError) * kD;
  rawTurn = pTerm + iTerm + dTerm;
  if (!isfinite(rawTurn)) rawTurn = 0.0f;

  turnPwm = (int)roundf(clampFloat(
    STEERING_DIRECTION * rawTurn,
    -(float)MAX_TURN_PWM,
    (float)MAX_TURN_PWM
  ));
  previousError = lineError;
}

// =================================================================================================
// MOTOR OUTPUT

void runMotorAtPwm(BoardMotor motor, int signedPwm) {
  int boundedPwm = clampInt(signedPwm, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
  int highPin = motor == MOTOR_1 ? M1H : M2H;
  int lowPin = motor == MOTOR_1 ? M1L : M2L;

  if (boundedPwm > 0) {
    ledcWrite(highPin, boundedPwm);
    ledcWrite(lowPin, 0);
  } else if (boundedPwm < 0) {
    ledcWrite(highPin, 0);
    ledcWrite(lowPin, abs(boundedPwm));
  } else {
    ledcWrite(highPin, 0);
    ledcWrite(lowPin, 0);
  }
}

void applyWheelPwm(int leftPwm, int rightPwm) {
  leftMotorPwm = leftPwm;
  rightMotorPwm = rightPwm;

  // Preserve the professor board's motor connector orientation: M1 is the right wheel, M2 left.
  runMotorAtPwm(MOTOR_1, rightMotorPwm);
  runMotorAtPwm(MOTOR_2, leftMotorPwm);
}

void stopMotors() {
  applyWheelPwm(0, 0);
}

void driveWithPid() {
  if (baseMotorPwm == 0) {
    stopMotors();
    return;
  }

  // Positive error means line right: left wheel faster, right wheel slower. Clamping both wheels
  // to the usable range keeps the rover moving forward rather than pivoting or reversing.
  int desiredLeft = clampInt(
    baseMotorPwm + turnPwm,
    MIN_EFFECTIVE_MOTOR_PWM,
    MAX_MOTOR_PWM
  );
  int desiredRight = clampInt(
    baseMotorPwm - turnPwm,
    MIN_EFFECTIVE_MOTOR_PWM,
    MAX_MOTOR_PWM
  );
  applyWheelPwm(desiredLeft, desiredRight);
}

// =================================================================================================
// TELEMETRY

void printTelemetry(unsigned long nowMs) {
  if (!ENABLE_SERIAL_TELEMETRY || nowMs - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = nowMs;

  Serial.print("speedAdc:");
  Serial.print(rawSpeedPot);
  Serial.print('\t');
  Serial.print("basePwm:");
  Serial.print(baseMotorPwm);
  Serial.print('\t');
  Serial.print("line:");
  Serial.print(lineDetected ? 1 : 0);
  Serial.print('\t');
  Serial.print("error:");
  Serial.print(lineError, 3);
  Serial.print('\t');
  Serial.print("P:");
  Serial.print(pTerm, 3);
  Serial.print('\t');
  Serial.print("I:");
  Serial.print(iTerm, 3);
  Serial.print('\t');
  Serial.print("D:");
  Serial.print(dTerm, 3);
  Serial.print('\t');
  Serial.print("turn:");
  Serial.print(turnPwm);
  Serial.print('\t');
  Serial.print("leftPwm:");
  Serial.print(leftMotorPwm);
  Serial.print('\t');
  Serial.print("rightPwm:");
  Serial.print(rightMotorPwm);
  Serial.print('\t');
  Serial.print("Kp:");
  Serial.print(kP, 3);
  Serial.print('\t');
  Serial.print("Ki:");
  Serial.print(kI, 4);
  Serial.print('\t');
  Serial.print("Kd:");
  Serial.println(kD, 3);
}

// =================================================================================================
// ARDUINO ENTRY POINTS

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  ledcAttach(M1H, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttach(M1L, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttach(M2H, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttach(M2L, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);

  for (int led = 0; led < LED_COUNT; led++) pinMode(LED_PINS[led], OUTPUT);

  stopMotors();
  calibrateSensors();
  readPotentiometers();
  resetPid();

  unsigned long nowMs = millis();
  lastControlMs = nowMs;
  lastTelemetryMs = nowMs;
}

void loop() {
  unsigned long nowMs = millis();
  if (nowMs - lastControlMs < CONTROL_INTERVAL_MS) {
    printTelemetry(nowMs);
    return;
  }
  lastControlMs = nowMs;

  readPotentiometers();
  readPhotoresistors();
  lineDetected = calculateLineError();

  if (!lineDetected || baseMotorPwm == 0) {
    stopMotors();
    resetPid();
  } else {
    calculatePidTurn();
    driveWithPid();
  }

  printTelemetry(nowMs);
}

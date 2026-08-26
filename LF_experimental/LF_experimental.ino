/*
  Experimental line follower derived from LF_original.ino.

  LF_original.ino is intentionally left unchanged. This sketch adds:
    - tunable PID behavior and derivative filtering
    - explicit line-detection thresholds
    - maximum turn and per-loop turn-increment limits
    - confidence-based speed control
    - weighted confidence, including consecutive successful PID checks
    - an in-memory action log for traceback

  Serial commands while running:
    t = stop briefly and dump the traceback log as CSV
    c = clear the traceback log
    h = print command help
*/

#include <Arduino.h>
#include <math.h>

// -------------------------------------------------------------------------------------------------
// Experimental settings. Start tuning here.

const bool PRINT_LIVE_DATA = false;
const bool ENABLE_ACTION_LOG = true;
const bool AUTO_DUMP_ON_LINE_LOSS = false;

const int NOMINAL_SPEED = 120;
const int SPEED_POT_ADDITION_MAX = 100;
const float LOW_CONFIDENCE_SPEED_SCALE = 0.35f;
const float FULL_CONFIDENCE_SPEED_SCALE = 1.00f;

const float KP_SCALE = 1.0f;
const float KI_SCALE = 0.001f;
const float KD_SCALE = 0.01f;
const float INTEGRAL_LIMIT = 5.0f;
const float DERIVATIVE_FILTER_ALPHA = 0.35f;

const float PEAK_TO_AVERAGE_RATIO = 1.50f;
const int MIN_LINE_VALUE = 25;
const int ACTIVE_SENSOR_THRESHOLD = 30;
const int MAX_ACTIVE_SENSORS = 4;
const float MIN_LOCAL_LINE_ENERGY = 35.0f;
const float MIN_CALIBRATION_SPAN = 50.0f;
const float MAX_ERROR_JUMP = 2.25f;

const float MIN_SENSOR_CONFIDENCE_FOR_PID = 0.35f;
const float MIN_LINE_CONFIDENCE = 0.60f;
const float LOW_CONFIDENCE_WARNING = 0.75f;
const int PID_CHECKS_FOR_FULL_CONFIDENCE = 6;

const float MAX_RAW_PID_TURN_FOR_CONFIDENCE = 220.0f;
const float MAX_TURN = 160.0f;
const float MAX_TURN_INCREMENT_PER_LOOP = 12.0f;

const unsigned long LIVE_PRINT_INTERVAL_MS = 100;
const int CONTROL_LOOP_DELAY_MS = 5;
const int ACTION_LOG_SIZE = 64;

// Confidence weights. The final score is earned weight / completed weight.
const float WEIGHT_CALIBRATION = 0.10f;
const float WEIGHT_PEAK = 0.20f;
const float WEIGHT_CONTRAST = 0.20f;
const float WEIGHT_ACTIVE_SENSOR_COUNT = 0.10f;
const float WEIGHT_LOCAL_ENERGY = 0.10f;
const float WEIGHT_ERROR_CONTINUITY = 0.10f;
const float WEIGHT_PID_OUTPUT = 0.10f;
const float WEIGHT_PID_HISTORY = 0.10f;

// -------------------------------------------------------------------------------------------------
// Hardware configuration copied from LF_original.ino.

enum Side { LEFT, RIGHT };
enum ActionType {
  ACTION_TRACKING,
  ACTION_LOW_CONFIDENCE,
  ACTION_TURN_LIMITED,
  ACTION_LINE_LOST
};

const int LDR_PINS[] = {1, 2, 3, 4, 5, 6, 7};
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

// -------------------------------------------------------------------------------------------------
// Runtime state.

int speedPotReading = 0;
int kPReading = 0;
int kIReading = 0;
int kDReading = 0;

float calibrationWhite[SENSOR_COUNT] = {0};
float calibrationBlack[SENSOR_COUNT] = {0};
float calibrationAccumulator[SENSOR_COUNT] = {0};
int normalizedLDR[SENSOR_COUNT] = {0};
int rawLDR[SENSOR_COUNT] = {0};

float kP = 0.0f;
float kI = 0.0f;
float kD = 0.0f;
float error = 0.0f;
float lastError = 0.0f;
float previousValidError = 0.0f;
float integralError = 0.0f;
float filteredDerivative = 0.0f;
float pTerm = 0.0f;
float iTerm = 0.0f;
float dTerm = 0.0f;
float rawTurn = 0.0f;
float appliedTurn = 0.0f;

float sensorConfidence = 0.0f;
float lineConfidence = 0.0f;
float completedConfidenceWeight = 0.0f;
float earnedConfidenceWeight = 0.0f;
int confidenceChecksCompleted = 0;
int confidenceChecksPassed = 0;
int consecutivePidChecks = 0;

int baseSpeed = 0;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int activeSensorCount = 0;
int peakSensorIndex = 0;
int peakSensorValue = 0;

bool hasCandidateError = false;
bool hasPreviousValidError = false;
bool lineDetected = false;
bool turnWasLimited = false;
bool hasEverDetectedLine = false;
bool previousLineDetected = false;
ActionType currentAction = ACTION_LINE_LOST;

unsigned long lastLivePrintMs = 0;

struct ActionLogEntry {
  unsigned long timestampMs;
  int sensorValues[SENSOR_COUNT];
  float error;
  float confidence;
  float pTerm;
  float iTerm;
  float dTerm;
  float requestedTurn;
  float appliedTurn;
  int baseSpeed;
  int leftMotorSpeed;
  int rightMotorSpeed;
  int checksPassed;
  int checksCompleted;
  int consecutivePidChecks;
  ActionType action;
};

ActionLogEntry actionLog[ACTION_LOG_SIZE];
int actionLogWriteIndex = 0;
int actionLogCount = 0;

// -------------------------------------------------------------------------------------------------
// Small helpers.

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

int clampInt(int value, int minimum, int maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

void addConfidenceScore(bool completed, float score, float weight) {
  if (!completed) return;

  float boundedScore = clampFloat(score, 0.0f, 1.0f);
  completedConfidenceWeight += weight;
  earnedConfidenceWeight += boundedScore * weight;
  confidenceChecksCompleted++;
  if (boundedScore >= 0.999f) confidenceChecksPassed++;
}

void addConfidenceCheck(bool completed, bool passed, float weight) {
  addConfidenceScore(completed, passed ? 1.0f : 0.0f, weight);
}

float calculateConfidence() {
  if (completedConfidenceWeight <= 0.0f) return 0.0f;
  return clampFloat(earnedConfidenceWeight / completedConfidenceWeight, 0.0f, 1.0f);
}

const char *actionName(ActionType action) {
  switch (action) {
    case ACTION_TRACKING: return "TRACKING";
    case ACTION_LOW_CONFIDENCE: return "LOW_CONFIDENCE";
    case ACTION_TURN_LIMITED: return "TURN_LIMITED";
    case ACTION_LINE_LOST: return "LINE_LOST";
    default: return "UNKNOWN";
  }
}

void setLeds(int value) {
  for (int i = 0; i < LED_COUNT; i++) digitalWrite(LED_PINS[i], value);
}

void runMotorAtSpeed(Side side, int speed) {
  int boundedSpeed = clampInt(speed, -255, 255);

  if (side == LEFT) {
    if (boundedSpeed > 0) {
      ledcWrite(M1H, boundedSpeed);
      ledcWrite(M1L, 0);
    } else {
      ledcWrite(M1H, 0);
      ledcWrite(M1L, abs(boundedSpeed));
    }
  } else {
    if (boundedSpeed > 0) {
      ledcWrite(M2H, boundedSpeed);
      ledcWrite(M2L, 0);
    } else {
      ledcWrite(M2H, 0);
      ledcWrite(M2L, abs(boundedSpeed));
    }
  }
}

void stopMotors() {
  runMotorAtSpeed(LEFT, 0);
  runMotorAtSpeed(RIGHT, 0);
}

// -------------------------------------------------------------------------------------------------
// Calibration and sensor input.

void calibrateHelper(int numberOfMeasurements, bool calibratingBlack) {
  Serial.println(calibratingBlack ? "\nCalibrating black" : "\nCalibrating white");

  for (int blink = 0; blink < 4; blink++) {
    setLeds(1);
    delay(250);
    setLeds(0);
    delay(250);
  }

  setLeds(1);
  delay(250);

  for (int measurement = 0; measurement < numberOfMeasurements; measurement++) {
    for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
      calibrationAccumulator[sensor] += (float)analogRead(LDR_PINS[sensor]);
      delay(2);
    }
    Serial.print(". ");
  }

  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    float average = roundf(calibrationAccumulator[sensor] / (float)numberOfMeasurements);
    if (calibratingBlack) calibrationBlack[sensor] = average;
    else calibrationWhite[sensor] = average;
    calibrationAccumulator[sensor] = 0.0f;
  }

  Serial.println("Done!");
  setLeds(0);
  delay(250);
}

void calibrate() {
  const int numberOfMeasurements = 20;
  calibrateHelper(numberOfMeasurements, false);
  setLeds(0);
  delay(2000);
  calibrateHelper(numberOfMeasurements, true);

  Serial.print("White values: ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(calibrationWhite[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("Black values: ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(calibrationBlack[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("Delta values: ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(calibrationBlack[i] - calibrationWhite[i]);
    Serial.print(' ');
  }
  Serial.println();

  setLeds(1);
  delay(2000);
}

int readMappedPotentiometer(int pin, int outputMinimum, int outputMaximum) {
  return map(analogRead(pin), 0, 4095, outputMinimum, outputMaximum);
}

void readPotentiometers() {
  speedPotReading = readMappedPotentiometer(SPEED_POT_PIN, 0, SPEED_POT_ADDITION_MAX);
  kPReading = readMappedPotentiometer(P_POT_PIN, 0, 100);
  kIReading = readMappedPotentiometer(I_POT_PIN, 0, 100);
  kDReading = readMappedPotentiometer(D_POT_PIN, 0, 100);
}

void readPhotoresistors() {
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    rawLDR[sensor] = analogRead(LDR_PINS[sensor]);
    float calibrationSpan = calibrationBlack[sensor] - calibrationWhite[sensor];

    if (fabsf(calibrationSpan) < MIN_CALIBRATION_SPAN) {
      normalizedLDR[sensor] = 0;
      continue;
    }

    float normalized = 100.0f * ((float)rawLDR[sensor] - calibrationWhite[sensor]) / calibrationSpan;
    normalizedLDR[sensor] = (int)roundf(clampFloat(normalized, 0.0f, 100.0f));
  }
}

// -------------------------------------------------------------------------------------------------
// Weighted line confidence and error calculation.

void calculateErrorAndSensorConfidence() {
  completedConfidenceWeight = 0.0f;
  earnedConfidenceWeight = 0.0f;
  confidenceChecksCompleted = 0;
  confidenceChecksPassed = 0;
  hasCandidateError = false;

  float average = 0.0f;
  peakSensorValue = -1;
  peakSensorIndex = 0;
  activeSensorCount = 0;
  int validCalibrationCount = 0;

  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    average += (float)normalizedLDR[sensor] / (float)SENSOR_COUNT;
    if (normalizedLDR[sensor] > peakSensorValue) {
      peakSensorValue = normalizedLDR[sensor];
      peakSensorIndex = sensor;
    }
    if (normalizedLDR[sensor] >= ACTIVE_SENSOR_THRESHOLD) activeSensorCount++;
    if (fabsf(calibrationBlack[sensor] - calibrationWhite[sensor]) >= MIN_CALIBRATION_SPAN) {
      validCalibrationCount++;
    }
  }

  addConfidenceScore(
    true,
    (float)validCalibrationCount / (float)SENSOR_COUNT,
    WEIGHT_CALIBRATION
  );
  addConfidenceCheck(true, peakSensorValue >= MIN_LINE_VALUE, WEIGHT_PEAK);
  addConfidenceCheck(
    true,
    average > 0.0f && (float)peakSensorValue >= PEAK_TO_AVERAGE_RATIO * average,
    WEIGHT_CONTRAST
  );
  addConfidenceCheck(
    true,
    activeSensorCount >= 1 && activeSensorCount <= MAX_ACTIVE_SENSORS,
    WEIGHT_ACTIVE_SENSOR_COUNT
  );

  int leftIndex = peakSensorIndex > 0 ? peakSensorIndex - 1 : peakSensorIndex;
  int rightIndex = peakSensorIndex < SENSOR_COUNT - 1 ? peakSensorIndex + 1 : peakSensorIndex;
  float localEnergy = (float)normalizedLDR[leftIndex]
                    + (float)normalizedLDR[peakSensorIndex]
                    + (float)normalizedLDR[rightIndex];
  addConfidenceCheck(true, localEnergy >= MIN_LOCAL_LINE_ENERGY, WEIGHT_LOCAL_ENERGY);

  if (localEnergy > 0.0f) {
    float weightedPosition =
      ((float)normalizedLDR[leftIndex] * (float)leftIndex
      + (float)normalizedLDR[peakSensorIndex] * (float)peakSensorIndex
      + (float)normalizedLDR[rightIndex] * (float)rightIndex) / localEnergy;

    float candidateError = weightedPosition - ((float)(SENSOR_COUNT - 1) / 2.0f);
    bool errorIsContinuous = !hasPreviousValidError
                          || fabsf(candidateError - previousValidError) <= MAX_ERROR_JUMP;
    addConfidenceCheck(hasPreviousValidError, errorIsContinuous, WEIGHT_ERROR_CONTINUITY);

    error = candidateError;
    hasCandidateError = true;
  } else {
    addConfidenceCheck(true, false, WEIGHT_ERROR_CONTINUITY);
  }

  sensorConfidence = calculateConfidence();
}

// -------------------------------------------------------------------------------------------------
// PID control, PID-based confidence, turn limits, and confidence-based speed.

void calculatePidTurn() {
  kP = (float)kPReading * KP_SCALE;
  kI = (float)kIReading * KI_SCALE;
  kD = (float)kDReading * KD_SCALE;

  float errorDelta = error - lastError;
  filteredDerivative = DERIVATIVE_FILTER_ALPHA * errorDelta
                     + (1.0f - DERIVATIVE_FILTER_ALPHA) * filteredDerivative;

  pTerm = error * kP;
  iTerm = integralError * kI;
  dTerm = filteredDerivative * kD;
  rawTurn = pTerm + iTerm + dTerm;

  bool pidHasUsableInput = hasCandidateError
                        && sensorConfidence >= MIN_SENSOR_CONFIDENCE_FOR_PID;
  bool pidOutputIsReasonable = pidHasUsableInput
                            && isfinite(rawTurn)
                            && fabsf(rawTurn) <= MAX_RAW_PID_TURN_FOR_CONFIDENCE;
  bool pidChangeIsReasonable = pidHasUsableInput && fabsf(errorDelta) <= MAX_ERROR_JUMP;

  addConfidenceCheck(true, pidOutputIsReasonable, WEIGHT_PID_OUTPUT);

  if (pidOutputIsReasonable && pidChangeIsReasonable) {
    if (consecutivePidChecks < PID_CHECKS_FOR_FULL_CONFIDENCE) consecutivePidChecks++;
  } else {
    consecutivePidChecks = 0;
  }

  float pidHistoryScore = (float)consecutivePidChecks / (float)PID_CHECKS_FOR_FULL_CONFIDENCE;
  addConfidenceScore(true, pidHistoryScore, WEIGHT_PID_HISTORY);
  lineConfidence = calculateConfidence();
  lineDetected = hasCandidateError && lineConfidence >= MIN_LINE_CONFIDENCE;

  if (lineDetected) {
    integralError = clampFloat(integralError + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    if (fabsf(error) < 0.01f) integralError = 0.0f;
    previousValidError = error;
    hasPreviousValidError = true;
    hasEverDetectedLine = true;
  } else {
    integralError *= 0.5f;
  }

  float requestedTurn = lineDetected ? clampFloat(rawTurn, -MAX_TURN, MAX_TURN) : 0.0f;
  float turnDelta = requestedTurn - appliedTurn;
  float limitedDelta = clampFloat(
    turnDelta,
    -MAX_TURN_INCREMENT_PER_LOOP,
    MAX_TURN_INCREMENT_PER_LOOP
  );
  turnWasLimited = fabsf(turnDelta - limitedDelta) > 0.001f
                || fabsf(rawTurn - requestedTurn) > 0.001f;
  appliedTurn = clampFloat(appliedTurn + limitedDelta, -MAX_TURN, MAX_TURN);

  if (hasCandidateError) lastError = error;
}

void runMotors() {
  int requestedCruiseSpeed = clampInt(NOMINAL_SPEED + speedPotReading, 0, 255);

  if (!lineDetected) {
    baseSpeed = 0;
  } else {
    float confidenceAboveThreshold = (lineConfidence - MIN_LINE_CONFIDENCE)
                                   / (1.0f - MIN_LINE_CONFIDENCE);
    float speedScale = LOW_CONFIDENCE_SPEED_SCALE
                     + clampFloat(confidenceAboveThreshold, 0.0f, 1.0f)
                     * (FULL_CONFIDENCE_SPEED_SCALE - LOW_CONFIDENCE_SPEED_SCALE);
    baseSpeed = (int)roundf((float)requestedCruiseSpeed * speedScale);
  }

  leftMotorSpeed = clampInt((int)roundf((float)baseSpeed - appliedTurn), -255, 255);
  rightMotorSpeed = clampInt((int)roundf((float)baseSpeed + appliedTurn), -255, 255);

  // Preserve the motor-to-pin orientation used by LF_original.ino.
  runMotorAtSpeed(LEFT, rightMotorSpeed);
  runMotorAtSpeed(RIGHT, leftMotorSpeed);

  if (!lineDetected) currentAction = ACTION_LINE_LOST;
  else if (turnWasLimited) currentAction = ACTION_TURN_LIMITED;
  else if (lineConfidence < LOW_CONFIDENCE_WARNING) currentAction = ACTION_LOW_CONFIDENCE;
  else currentAction = ACTION_TRACKING;
}

// -------------------------------------------------------------------------------------------------
// Traceback action log.

void recordAction() {
  if (!ENABLE_ACTION_LOG) return;

  ActionLogEntry &entry = actionLog[actionLogWriteIndex];
  entry.timestampMs = millis();
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    entry.sensorValues[sensor] = normalizedLDR[sensor];
  }
  entry.error = error;
  entry.confidence = lineConfidence;
  entry.pTerm = pTerm;
  entry.iTerm = iTerm;
  entry.dTerm = dTerm;
  entry.requestedTurn = rawTurn;
  entry.appliedTurn = appliedTurn;
  entry.baseSpeed = baseSpeed;
  entry.leftMotorSpeed = leftMotorSpeed;
  entry.rightMotorSpeed = rightMotorSpeed;
  entry.checksPassed = confidenceChecksPassed;
  entry.checksCompleted = confidenceChecksCompleted;
  entry.consecutivePidChecks = consecutivePidChecks;
  entry.action = currentAction;

  actionLogWriteIndex = (actionLogWriteIndex + 1) % ACTION_LOG_SIZE;
  if (actionLogCount < ACTION_LOG_SIZE) actionLogCount++;
}

void clearActionLog() {
  actionLogWriteIndex = 0;
  actionLogCount = 0;
  Serial.println("Traceback log cleared.");
}

void dumpActionLog() {
  stopMotors();
  Serial.println("TRACEBACK_BEGIN");
  Serial.print("ms,action,error,confidence,p,i,d,raw_turn,limited_turn,base_speed,left_speed,right_speed,");
  Serial.print("checks_passed,checks_completed,consecutive_pid_checks");
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    Serial.print(",ldr");
    Serial.print(sensor);
  }
  Serial.println();

  int oldestIndex = actionLogCount == ACTION_LOG_SIZE ? actionLogWriteIndex : 0;
  for (int offset = 0; offset < actionLogCount; offset++) {
    const ActionLogEntry &entry = actionLog[(oldestIndex + offset) % ACTION_LOG_SIZE];
    Serial.print(entry.timestampMs);
    Serial.print(',');
    Serial.print(actionName(entry.action));
    Serial.print(',');
    Serial.print(entry.error, 4);
    Serial.print(',');
    Serial.print(entry.confidence, 4);
    Serial.print(',');
    Serial.print(entry.pTerm, 4);
    Serial.print(',');
    Serial.print(entry.iTerm, 4);
    Serial.print(',');
    Serial.print(entry.dTerm, 4);
    Serial.print(',');
    Serial.print(entry.requestedTurn, 4);
    Serial.print(',');
    Serial.print(entry.appliedTurn, 4);
    Serial.print(',');
    Serial.print(entry.baseSpeed);
    Serial.print(',');
    Serial.print(entry.leftMotorSpeed);
    Serial.print(',');
    Serial.print(entry.rightMotorSpeed);
    Serial.print(',');
    Serial.print(entry.checksPassed);
    Serial.print(',');
    Serial.print(entry.checksCompleted);
    Serial.print(',');
    Serial.print(entry.consecutivePidChecks);
    for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
      Serial.print(',');
      Serial.print(entry.sensorValues[sensor]);
    }
    Serial.println();
  }
  Serial.println("TRACEBACK_END");
}

void printSerialHelp() {
  Serial.println("Commands: t = dump traceback CSV, c = clear traceback, h = help");
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = (char)Serial.read();
    if (command == 't' || command == 'T') dumpActionLog();
    else if (command == 'c' || command == 'C') clearActionLog();
    else if (command == 'h' || command == 'H') printSerialHelp();
  }
}

void maybeDumpTracebackOnLoss() {
  bool justLostLine = hasEverDetectedLine && previousLineDetected && !lineDetected;
  if (AUTO_DUMP_ON_LINE_LOSS && justLostLine) dumpActionLog();
  previousLineDetected = lineDetected;
}

void printLiveData() {
  if (!PRINT_LIVE_DATA || millis() - lastLivePrintMs < LIVE_PRINT_INTERVAL_MS) return;
  lastLivePrintMs = millis();

  Serial.print("Action: ");
  Serial.print(actionName(currentAction));
  Serial.print(" confidence: ");
  Serial.print(lineConfidence, 3);
  Serial.print(" checks: ");
  Serial.print(confidenceChecksPassed);
  Serial.print('/');
  Serial.print(confidenceChecksCompleted);
  Serial.print(" PID checks: ");
  Serial.print(consecutivePidChecks);
  Serial.print(" error: ");
  Serial.print(error, 3);
  Serial.print(" turn raw/applied: ");
  Serial.print(rawTurn, 2);
  Serial.print('/');
  Serial.print(appliedTurn, 2);
  Serial.print(" speed base/L/R: ");
  Serial.print(baseSpeed);
  Serial.print('/');
  Serial.print(leftMotorSpeed);
  Serial.print('/');
  Serial.println(rightMotorSpeed);
}

// -------------------------------------------------------------------------------------------------
// Arduino entry points.

void setup() {
  Serial.begin(9600);
  ledcAttach(M1H, 12000, 8);
  ledcAttach(M1L, 12000, 8);
  ledcAttach(M2H, 12000, 8);
  ledcAttach(M2L, 12000, 8);

  for (int i = 0; i < LED_COUNT; i++) pinMode(LED_PINS[i], OUTPUT);

  calibrate();
  readPotentiometers();
  printSerialHelp();
}

void loop() {
  handleSerialCommands();
  readPotentiometers();
  readPhotoresistors();
  calculateErrorAndSensorConfidence();
  calculatePidTurn();
  runMotors();
  recordAction();
  maybeDumpTracebackOnLoss();
  printLiveData();
  delay(CONTROL_LOOP_DELAY_MS);
}

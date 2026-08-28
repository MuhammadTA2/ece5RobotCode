/* *************************************************************************************************
   UCSD ECE 5 Lab 4: Experimental PID Line-Following Robot
   Edited by Muhammad Abouelkhir

   LF_original.ino is the unchanged reference implementation. This file is the experimental sketch.

   FEATURES
   - Floating-point, potentiometer-controlled PID with a fixed-time control loop
   - Documented calibration and test values in one settings area
   - Weighted line confidence and confidence-based speed reduction
   - PID anti-windup, derivative filtering, turn clamping, and turn slew limiting
   - Motor mixing that preserves steering instead of clipping one saturated wheel
   - Measured motor deadband compensation so nonzero commands produce usable wheel motion
   - Confirmed line loss stops the motors; confirmed line reacquisition restarts normal PID control
   - Serial Plotter telemetry plus optional oscilloscope debug PWM outputs

   RECOMMENDED TEST ORDER
   1. Raise the wheels. Confirm forward, left, right, stop, and reverse motor directions.
   2. Calibrate white and black, then inspect the printed calibration spans.
   3. Tune BLACK_DETECTION_THRESHOLD at low speed.
   4. Set Ki and Kd to zero. Raise Kp until oscillation starts, then reduce Kp 20-30%.
   5. Add Kd until oscillation is damped. Add only enough Ki to remove persistent bias.
   6. Tune speed, turn limits, and confidence.

   POTENTIOMETER MATH
     fraction = filtered ADC / 4095
     gain = GAIN_MIN + fraction * (GAIN_MAX - GAIN_MIN)
     ADC for a desired gain = 4095 * (gain - GAIN_MIN) / (GAIN_MAX - GAIN_MIN)

   OSCILLOSCOPE SAFETY
   Do not attach a normal grounded probe across H-bridge motor terminals. The optional scope outputs
   below are logic-level debug signals referenced to ESP32 ground. Add an RC low-pass filter if an
   analog-looking error/turn trace is desired.
************************************************************************************************* */

#include <Arduino.h>
#include <math.h>

// =================================================================================================
// ADJUSTABLE SETTINGS: change these during calibration and robot testing.
// Hardware pins are in a separate section and should change only when the robot is rewired.

// --- SENSOR CALIBRATION ---------------------------------------------------------------------------

// More measurements reject noise but make calibration slower. Suggested range: 10-50.
const int CALIBRATION_MEASUREMENTS = 20;

// Pause between white and black calibration. Increase if more setup time is needed.
const unsigned long CALIBRATION_COLOR_CHANGE_MS = 2000;

// Minimum absolute raw ADC difference between calibrated white and black.
// Raise to reject poorly calibrated sensors; lower only if all real sensor spans are small.
// Suggested starting range: 30-150 ADC counts.
const float MIN_CALIBRATION_SPAN = 50.0f;

// Normalized 0-100 value that means a sensor sees black.
// Raise if the floor causes false black readings; lower if the robot loses a faint black line.
// Suggested starting range: 20-45.
const int BLACK_DETECTION_THRESHOLD = 30;

// A strong peak should exceed the sensor-array average by this ratio.
// Raise to demand a clearer/narrower line; lower for wide or low-contrast lines.
const float PEAK_TO_AVERAGE_RATIO = 1.50f;

// Used to judge how many sensors simultaneously see black.
const int ACTIVE_SENSOR_THRESHOLD = 30;
const int MAX_EXPECTED_ACTIVE_SENSORS = 5;

// Minimum summed normalized signal around the strongest sensor.
const float MIN_LOCAL_LINE_ENERGY = 35.0f;

// Maximum plausible change in calculated line position from one 10 ms control sample to the next.
const float MAX_ERROR_JUMP = 2.25f;

// --- POTENTIOMETER GAIN RANGES --------------------------------------------------------------------

// The old Kp range reached 100, which could request about 300 PWM at maximum line error.
// With MAX_TURN=160 and error near 3, Kp around 53 already uses the entire turn budget.
const float KP_MIN = 0.0f;
const float KP_MAX = 50.0f;

// The PID uses error-seconds for its integral, so this range is meaningful at any loop rate.
// Raise KI_MAX only if the I knob cannot remove a persistent left/right bias.
const float KI_MIN = 0.0f;
const float KI_MAX = 12.0f;

// The PID derivative is error-per-second. Raise KD_MAX if the D knob cannot damp oscillation;
// lower it if a small knob movement makes steering noisy.
const float KD_MIN = 0.0f;
const float KD_MAX = 1.50f;

// Smooths ADC noise from all four potentiometers. Higher reacts faster but is noisier.
// Suggested range: 0.05-0.30.
const float POTENTIOMETER_FILTER_ALPHA = 0.15f;

// Treat the bottom of a PID potentiometer as exactly zero despite ADC noise.
const int POTENTIOMETER_ZERO_DEAD_ZONE_ADC = 30;

// --- PID, SPEED, AND STEERING ---------------------------------------------------------------------

// Fixed control timing makes Ki and Kd repeatable. Suggested range: 5-20 ms.
const unsigned long CONTROL_INTERVAL_MS = 10;

// Starting forward PWM plus 0-SPEED_POT_ADDITION_MAX from the speed potentiometer.
// Increase gradually. Lower speeds make PID calibration safer.
const int NOMINAL_SPEED = 110;
const int SPEED_POT_ADDITION_MAX = 100;
const int MAX_REQUESTED_CRUISE_SPEED = 220;

// Bench testing found that all four motor directions start reliably at PWM 170. A 5-count margin
// is included here. Re-test after changing motors, battery, wheels, or gearing, then replace 175
// with the lowest value that starts every motor direction reliably.
const int MIN_EFFECTIVE_MOTOR_PWM = 175;

// Logical PID commands at or below this magnitude are treated as a true stop. Increase if PID
// noise makes a nearly stopped wheel chatter between forward and reverse; lower for finer pivots.
const int MOTOR_COMMAND_STOP_BAND = 10;

// At low confidence the robot retains this fraction of requested cruise speed.
// Lower for caution; raise if the robot stalls before it can correct.
const float LOW_CONFIDENCE_SPEED_SCALE = 0.35f;
const float FULL_CONFIDENCE_SPEED_SCALE = 1.00f;

// Absolute PID steering correction in PWM units.
const float MAX_TURN = 160.0f;

// Sensor pins are ordered left-to-right, so a positive error means the line is to the robot's
// right. +1 makes that error speed up the left wheel and slow the right wheel. Change to -1 only
// if different motor or sensor wiring makes the robot steer in the opposite direction.
const float STEERING_DIRECTION = 1.0f;

// Maximum change in steering correction per second. Lower makes steering smoother but slower.
const float MAX_TURN_CHANGE_PER_SECOND = 1200.0f;

// Maximum accumulated error-seconds. Lower reduces windup and long-lasting steering bias.
const float INTEGRAL_ERROR_LIMIT = 3.0f;

// Derivative low-pass time constant. Raise to reduce noise; lower for a faster D response.
const float DERIVATIVE_FILTER_TIME_CONSTANT_SECONDS = 0.040f;

// --- CONFIDENCE -----------------------------------------------------------------------------------

// Confidence below this value uses the minimum speed scale. Complete line loss is handled
// separately and stops both motors.
const float MIN_LINE_CONFIDENCE = 0.60f;

// PID must be reasonable for this many consecutive cycles to earn full PID-history confidence.
const int PID_CHECKS_FOR_FULL_CONFIDENCE = 6;

// A very large raw PID request reduces confidence before output limiting.
const float MAX_RAW_PID_TURN_FOR_CONFIDENCE = 220.0f;

// Maximum error velocity considered stable for the PID-history confidence check.
const float MAX_ERROR_RATE_FOR_STABLE_PID = 150.0f;

// Confidence is earned weight / completed weight. These do not have to total exactly one.
const float WEIGHT_CALIBRATION = 0.10f;
const float WEIGHT_PEAK = 0.20f;
const float WEIGHT_CONTRAST = 0.15f;
const float WEIGHT_ACTIVE_SENSOR_COUNT = 0.10f;
const float WEIGHT_LOCAL_ENERGY = 0.10f;
const float WEIGHT_ERROR_CONTINUITY = 0.10f;
const float WEIGHT_PID_OUTPUT = 0.10f;
const float WEIGHT_PID_HISTORY = 0.15f;

// --- LINE-LOSS SAFETY -----------------------------------------------------------------------------

// Three no-black samples at a 10 ms control interval confirm complete line loss.
const int LINE_LOST_CONFIRMATION_SAMPLES = 3;

// Require two consecutive black samples before normal PID control resumes after any line loss.
const int LINE_REACQUIRE_CONFIRMATION_SAMPLES = 2;

// --- TELEMETRY AND OPTIONAL SCOPE OUTPUTS ---------------------------------------------------------

// Serial Plotter is the recommended tuning tool. At 115200 baud, 20 Hz telemetry is non-blocking
// enough for this controller. Turn it off for final untethered speed tests.
const bool ENABLE_SERIAL_TELEMETRY = true;
const unsigned long TELEMETRY_INTERVAL_MS = 50;
const unsigned long SERIAL_BAUD_RATE = 115200;

// Optional scope PWM outputs are disabled until unused, output-capable GPIOs are assigned.
// After an RC filter: 0 V = negative full scale, ~1.65 V = zero, ~3.3 V = positive full scale.
const bool ENABLE_SCOPE_DEBUG = false;
const int SCOPE_ERROR_PIN = -1;
const int SCOPE_TURN_PIN = -1;
const int SCOPE_SYNC_PIN = -1;
const int SCOPE_PWM_FREQUENCY = 2000;
const int SCOPE_PWM_RESOLUTION_BITS = 8;

// =================================================================================================
// HARDWARE SETTINGS: do not change these during normal calibration.

enum MotorSide { LEFT_MOTOR, RIGHT_MOTOR };

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

const int MOTOR_PWM_FREQUENCY = 12000;
const int MOTOR_PWM_RESOLUTION_BITS = 8;

// =================================================================================================
// Runtime state. These values are not calibration settings.

enum ControlState {
  STATE_TRACKING = 0,
  STATE_LOSS_PENDING = 1,
  STATE_LINE_LOST = 2,
  STATE_REACQUIRE_VERIFY = 3
};

struct ConfidenceAccumulator {
  float earnedWeight;
  float completedWeight;
  int checksPassed;
  int checksCompleted;
};

float calibrationWhite[SENSOR_COUNT] = {0};
float calibrationBlack[SENSOR_COUNT] = {0};
float calibrationAccumulator[SENSOR_COUNT] = {0};
int rawLdr[SENSOR_COUNT] = {0};
int normalizedLdr[SENSOR_COUNT] = {0};

float filteredSpeedAdc = 0.0f;
float filteredPAdc = 0.0f;
float filteredIAdc = 0.0f;
float filteredDAdc = 0.0f;
bool potentiometersInitialized = false;

int speedPotAddition = 0;
float kP = 0.0f;
float kI = 0.0f;
float kD = 0.0f;

float lineError = 0.0f;
float previousPidError = 0.0f;
float previousValidLineError = 0.0f;
float integralError = 0.0f;
float filteredDerivative = 0.0f;
float pTerm = 0.0f;
float iTerm = 0.0f;
float dTerm = 0.0f;
float rawTurn = 0.0f;
float appliedTurn = 0.0f;
bool hasPidErrorHistory = false;
bool hasPreviousValidLineError = false;
bool turnWasLimited = false;
bool outputWasSaturated = false;

int validCalibrationCount = 0;
int activeSensorCount = 0;
int peakSensorIndex = 0;
int peakSensorValue = 0;
bool blackDetected = false;
bool hasCandidateError = false;

ConfidenceAccumulator confidenceAccumulator = {0.0f, 0.0f, 0, 0};
float sensorConfidence = 0.0f;
float lineConfidence = 0.0f;
int consecutivePidChecks = 0;

int baseSpeed = 0;
// These are post-PID logical commands in -255..255 units, before motor deadband compensation.
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
// These are the signed PWM values actually sent to the physical left and right motors.
int leftAppliedPwm = 0;
int rightAppliedPwm = 0;

ControlState controlState = STATE_TRACKING;
int consecutiveLostSamples = 0;
int consecutiveReacquiredSamples = 0;

unsigned long lastControlMs = 0;
unsigned long lastTelemetryMs = 0;

// =================================================================================================
// General helpers.

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

const char *stateName(ControlState state) {
  switch (state) {
    case STATE_TRACKING: return "TRACKING";
    case STATE_LOSS_PENDING: return "LOSS_PENDING";
    case STATE_LINE_LOST: return "LINE_LOST";
    case STATE_REACQUIRE_VERIFY: return "REACQUIRE_VERIFY";
    default: return "UNKNOWN";
  }
}

void setLeds(int value) {
  for (int i = 0; i < LED_COUNT; i++) digitalWrite(LED_PINS[i], value);
}

void setControlState(ControlState nextState) {
  controlState = nextState;
}

// =================================================================================================
// Motor output.

int compensateMotorDeadband(int logicalSpeed) {
  int boundedCommand = clampInt(logicalSpeed, -255, 255);
  int commandMagnitude = abs(boundedCommand);

  if (commandMagnitude <= MOTOR_COMMAND_STOP_BAND) return 0;

  float commandProgress = (float)(commandMagnitude - MOTOR_COMMAND_STOP_BAND)
                          / (float)(255 - MOTOR_COMMAND_STOP_BAND);
  int physicalMagnitude = (int)roundf(
    (float)MIN_EFFECTIVE_MOTOR_PWM
    + commandProgress * (float)(255 - MIN_EFFECTIVE_MOTOR_PWM)
  );

  return boundedCommand < 0 ? -physicalMagnitude : physicalMagnitude;
}

void runMotorAtSpeed(MotorSide side, int speed) {
  int boundedSpeed = clampInt(speed, -255, 255);

  if (side == LEFT_MOTOR) {
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

void applyMotorSpeeds(int logicalLeftSpeed, int logicalRightSpeed) {
  leftMotorSpeed = clampInt(logicalLeftSpeed, -255, 255);
  rightMotorSpeed = clampInt(logicalRightSpeed, -255, 255);
  leftAppliedPwm = compensateMotorDeadband(leftMotorSpeed);
  rightAppliedPwm = compensateMotorDeadband(rightMotorSpeed);

  // Preserve the original sketch's physical motor-to-pin orientation.
  runMotorAtSpeed(LEFT_MOTOR, rightAppliedPwm);
  runMotorAtSpeed(RIGHT_MOTOR, leftAppliedPwm);
}

void stopMotors() {
  applyMotorSpeeds(0, 0);
  baseSpeed = 0;
}

// =================================================================================================
// Calibration and analog input.

void calibrateColor(bool calibratingBlack) {
  Serial.println(calibratingBlack ? "\nCalibrating black" : "\nCalibrating white");

  for (int blink = 0; blink < 4; blink++) {
    setLeds(1);
    delay(250);
    setLeds(0);
    delay(250);
  }

  setLeds(1);
  delay(250);

  for (int measurement = 0; measurement < CALIBRATION_MEASUREMENTS; measurement++) {
    for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
      calibrationAccumulator[sensor] += (float)analogRead(LDR_PINS[sensor]);
      delay(2);
    }
    Serial.print(". ");
  }

  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    float average = roundf(calibrationAccumulator[sensor] / (float)CALIBRATION_MEASUREMENTS);
    if (calibratingBlack) calibrationBlack[sensor] = average;
    else calibrationWhite[sensor] = average;
    calibrationAccumulator[sensor] = 0.0f;
  }

  Serial.println("Done!");
  setLeds(0);
  delay(250);
}

void printCalibrationValues() {
  Serial.print("White values: ");
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    Serial.print(calibrationWhite[sensor]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("Black values: ");
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    Serial.print(calibrationBlack[sensor]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("Absolute spans: ");
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    Serial.print(fabsf(calibrationBlack[sensor] - calibrationWhite[sensor]));
    Serial.print(' ');
  }
  Serial.println();
}

void calibrateSensors() {
  calibrateColor(false);
  setLeds(0);
  delay(CALIBRATION_COLOR_CHANGE_MS);
  calibrateColor(true);
  printCalibrationValues();
  setLeds(1);
  delay(CALIBRATION_COLOR_CHANGE_MS);
}

float updateFilteredAdc(float previousValue, int rawValue) {
  return previousValue + POTENTIOMETER_FILTER_ALPHA * ((float)rawValue - previousValue);
}

float adcFractionWithDeadZone(float adcValue, bool useZeroDeadZone) {
  float lowerBound = useZeroDeadZone ? (float)POTENTIOMETER_ZERO_DEAD_ZONE_ADC : 0.0f;
  if (adcValue <= lowerBound) return 0.0f;
  return clampFloat((adcValue - lowerBound) / (4095.0f - lowerBound), 0.0f, 1.0f);
}

float mapFractionToRange(float fraction, float minimum, float maximum) {
  return minimum + clampFloat(fraction, 0.0f, 1.0f) * (maximum - minimum);
}

void readPotentiometers() {
  int rawSpeed = analogRead(SPEED_POT_PIN);
  int rawP = analogRead(P_POT_PIN);
  int rawI = analogRead(I_POT_PIN);
  int rawD = analogRead(D_POT_PIN);

  if (!potentiometersInitialized) {
    filteredSpeedAdc = (float)rawSpeed;
    filteredPAdc = (float)rawP;
    filteredIAdc = (float)rawI;
    filteredDAdc = (float)rawD;
    potentiometersInitialized = true;
  } else {
    filteredSpeedAdc = updateFilteredAdc(filteredSpeedAdc, rawSpeed);
    filteredPAdc = updateFilteredAdc(filteredPAdc, rawP);
    filteredIAdc = updateFilteredAdc(filteredIAdc, rawI);
    filteredDAdc = updateFilteredAdc(filteredDAdc, rawD);
  }

  speedPotAddition = (int)roundf(
    adcFractionWithDeadZone(filteredSpeedAdc, false) * (float)SPEED_POT_ADDITION_MAX
  );
  kP = mapFractionToRange(adcFractionWithDeadZone(filteredPAdc, true), KP_MIN, KP_MAX);
  kI = mapFractionToRange(adcFractionWithDeadZone(filteredIAdc, true), KI_MIN, KI_MAX);
  kD = mapFractionToRange(adcFractionWithDeadZone(filteredDAdc, true), KD_MIN, KD_MAX);
}

void readPhotoresistors() {
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    rawLdr[sensor] = analogRead(LDR_PINS[sensor]);
    float span = calibrationBlack[sensor] - calibrationWhite[sensor];

    if (fabsf(span) < MIN_CALIBRATION_SPAN) {
      normalizedLdr[sensor] = 0;
      continue;
    }

    // Signed span supports either electrical polarity: calibrated black always maps toward 100.
    float normalized = 100.0f * ((float)rawLdr[sensor] - calibrationWhite[sensor]) / span;
    normalizedLdr[sensor] = (int)roundf(clampFloat(normalized, 0.0f, 100.0f));
  }
}

// =================================================================================================
// Confidence and line-position calculation.

void resetConfidenceAccumulator() {
  confidenceAccumulator.earnedWeight = 0.0f;
  confidenceAccumulator.completedWeight = 0.0f;
  confidenceAccumulator.checksPassed = 0;
  confidenceAccumulator.checksCompleted = 0;
}

void addConfidenceScore(bool completed, float score, float weight) {
  if (!completed) return;

  float boundedScore = clampFloat(score, 0.0f, 1.0f);
  confidenceAccumulator.completedWeight += weight;
  confidenceAccumulator.earnedWeight += boundedScore * weight;
  confidenceAccumulator.checksCompleted++;
  if (boundedScore >= 0.999f) confidenceAccumulator.checksPassed++;
}

void addConfidenceCheck(bool completed, bool passed, float weight) {
  addConfidenceScore(completed, passed ? 1.0f : 0.0f, weight);
}

float currentConfidence() {
  if (confidenceAccumulator.completedWeight <= 0.0f) return 0.0f;
  return clampFloat(
    confidenceAccumulator.earnedWeight / confidenceAccumulator.completedWeight,
    0.0f,
    1.0f
  );
}

void calculateLineErrorAndSensorConfidence() {
  resetConfidenceAccumulator();
  blackDetected = false;
  hasCandidateError = false;
  validCalibrationCount = 0;
  activeSensorCount = 0;
  peakSensorIndex = 0;
  peakSensorValue = -1;

  float average = 0.0f;
  float totalSignal = 0.0f;
  float weightedPositionSum = 0.0f;

  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    float span = fabsf(calibrationBlack[sensor] - calibrationWhite[sensor]);
    bool calibrationValid = span >= MIN_CALIBRATION_SPAN;
    if (calibrationValid) validCalibrationCount++;

    int value = normalizedLdr[sensor];
    average += (float)value / (float)SENSOR_COUNT;
    totalSignal += (float)value;
    weightedPositionSum += (float)value * (float)sensor;

    if (value > peakSensorValue) {
      peakSensorValue = value;
      peakSensorIndex = sensor;
    }
    if (value >= ACTIVE_SENSOR_THRESHOLD) activeSensorCount++;
    if (calibrationValid && value >= BLACK_DETECTION_THRESHOLD) blackDetected = true;
  }

  addConfidenceScore(
    true,
    (float)validCalibrationCount / (float)SENSOR_COUNT,
    WEIGHT_CALIBRATION
  );
  addConfidenceCheck(true, peakSensorValue >= BLACK_DETECTION_THRESHOLD, WEIGHT_PEAK);
  addConfidenceCheck(
    true,
    average > 0.0f && (float)peakSensorValue >= PEAK_TO_AVERAGE_RATIO * average,
    WEIGHT_CONTRAST
  );
  addConfidenceCheck(
    true,
    activeSensorCount >= 1 && activeSensorCount <= MAX_EXPECTED_ACTIVE_SENSORS,
    WEIGHT_ACTIVE_SENSOR_COUNT
  );

  int firstNeighbor = max(0, peakSensorIndex - 1);
  int lastNeighbor = min(SENSOR_COUNT - 1, peakSensorIndex + 1);
  float localEnergy = 0.0f;
  for (int sensor = firstNeighbor; sensor <= lastNeighbor; sensor++) {
    localEnergy += (float)normalizedLdr[sensor];
  }
  addConfidenceCheck(true, localEnergy >= MIN_LOCAL_LINE_ENERGY, WEIGHT_LOCAL_ENERGY);

  if (totalSignal > 0.0f) {
    float weightedPosition = weightedPositionSum / totalSignal;
    float candidateError = weightedPosition - ((float)(SENSOR_COUNT - 1) / 2.0f);
    bool errorIsContinuous = !hasPreviousValidLineError
                          || fabsf(candidateError - previousValidLineError) <= MAX_ERROR_JUMP;
    addConfidenceCheck(
      hasPreviousValidLineError,
      errorIsContinuous,
      WEIGHT_ERROR_CONTINUITY
    );
    lineError = candidateError;
    hasCandidateError = true;
  } else {
    addConfidenceCheck(true, false, WEIGHT_ERROR_CONTINUITY);
  }

  sensorConfidence = currentConfidence();
  lineConfidence = sensorConfidence;
}

// =================================================================================================
// PID and confidence-based tracking control.

void resetPidState() {
  previousPidError = lineError;
  integralError = 0.0f;
  filteredDerivative = 0.0f;
  pTerm = 0.0f;
  iTerm = 0.0f;
  dTerm = 0.0f;
  rawTurn = 0.0f;
  appliedTurn = 0.0f;
  hasPidErrorHistory = false;
  hasPreviousValidLineError = false;
  consecutivePidChecks = 0;
  turnWasLimited = false;
  outputWasSaturated = false;
}

void calculatePidAndConfidence(float dtSeconds) {
  bool pidHasInput = blackDetected && hasCandidateError;
  float errorRate = 0.0f;

  if (pidHasInput && hasPidErrorHistory && dtSeconds > 0.0f) {
    errorRate = (lineError - previousPidError) / dtSeconds;
  }

  float derivativeAlpha = dtSeconds / (DERIVATIVE_FILTER_TIME_CONSTANT_SECONDS + dtSeconds);
  derivativeAlpha = clampFloat(derivativeAlpha, 0.0f, 1.0f);
  filteredDerivative += derivativeAlpha * (errorRate - filteredDerivative);

  float proposedIntegral = clampFloat(
    integralError + lineError * dtSeconds,
    -INTEGRAL_ERROR_LIMIT,
    INTEGRAL_ERROR_LIMIT
  );

  pTerm = pidHasInput ? lineError * kP : 0.0f;
  dTerm = pidHasInput ? filteredDerivative * kD : 0.0f;
  float proposedITerm = pidHasInput ? proposedIntegral * kI : 0.0f;
  float proposedRawTurn = pTerm + proposedITerm + dTerm;

  // Conditional integration: do not accumulate error if it pushes farther into saturation.
  bool proposedHighSaturation = proposedRawTurn > MAX_TURN && lineError > 0.0f;
  bool proposedLowSaturation = proposedRawTurn < -MAX_TURN && lineError < 0.0f;
  if (pidHasInput && !proposedHighSaturation && !proposedLowSaturation) {
    integralError = proposedIntegral;
  } else if (!pidHasInput) {
    integralError *= 0.5f;
  }

  iTerm = pidHasInput ? integralError * kI : 0.0f;
  rawTurn = pTerm + iTerm + dTerm;
  if (!isfinite(rawTurn)) rawTurn = 0.0f;

  bool pidOutputReasonable = pidHasInput
                          && fabsf(rawTurn) <= MAX_RAW_PID_TURN_FOR_CONFIDENCE;
  bool pidMotionStable = pidHasInput
                      && (!hasPidErrorHistory || fabsf(errorRate) <= MAX_ERROR_RATE_FOR_STABLE_PID);
  addConfidenceCheck(true, pidOutputReasonable, WEIGHT_PID_OUTPUT);

  if (pidOutputReasonable && pidMotionStable) {
    if (consecutivePidChecks < PID_CHECKS_FOR_FULL_CONFIDENCE) consecutivePidChecks++;
  } else {
    consecutivePidChecks = 0;
  }

  float pidHistoryScore = (float)consecutivePidChecks / (float)PID_CHECKS_FOR_FULL_CONFIDENCE;
  addConfidenceScore(true, pidHistoryScore, WEIGHT_PID_HISTORY);
  lineConfidence = currentConfidence();

  float requestedTurn = clampFloat(rawTurn, -MAX_TURN, MAX_TURN);
  float maximumTurnStep = MAX_TURN_CHANGE_PER_SECOND * dtSeconds;
  float turnDelta = requestedTurn - appliedTurn;
  float limitedTurnDelta = clampFloat(turnDelta, -maximumTurnStep, maximumTurnStep);
  appliedTurn = clampFloat(appliedTurn + limitedTurnDelta, -MAX_TURN, MAX_TURN);

  turnWasLimited = fabsf(turnDelta - limitedTurnDelta) > 0.001f
                || fabsf(rawTurn - requestedTurn) > 0.001f;

  previousPidError = lineError;
  hasPidErrorHistory = pidHasInput;
  if (pidHasInput) {
    previousValidLineError = lineError;
    hasPreviousValidLineError = true;
  }
}

void driveWithPidAndConfidence(float dtSeconds) {
  calculatePidAndConfidence(dtSeconds);

  int requestedCruiseSpeed = clampInt(
    NOMINAL_SPEED + speedPotAddition,
    0,
    MAX_REQUESTED_CRUISE_SPEED
  );

  float confidenceProgress = (lineConfidence - MIN_LINE_CONFIDENCE)
                           / (1.0f - MIN_LINE_CONFIDENCE);
  float speedScale = LOW_CONFIDENCE_SPEED_SCALE
                   + clampFloat(confidenceProgress, 0.0f, 1.0f)
                   * (FULL_CONFIDENCE_SPEED_SCALE - LOW_CONFIDENCE_SPEED_SCALE);
  int confidenceSpeed = (int)roundf((float)requestedCruiseSpeed * speedScale);

  // Reserve PWM headroom for steering. This preserves left/right difference instead of clipping
  // only the faster wheel when baseSpeed +/- appliedTurn would exceed the motor limit.
  int maximumBaseForTurn = max(0, 255 - (int)ceilf(fabsf(appliedTurn)));
  baseSpeed = min(confidenceSpeed, maximumBaseForTurn);
  outputWasSaturated = confidenceSpeed > maximumBaseForTurn || fabsf(rawTurn) > MAX_TURN;

  float directedTurn = STEERING_DIRECTION * appliedTurn;
  int desiredLeft = (int)roundf((float)baseSpeed + directedTurn);
  int desiredRight = (int)roundf((float)baseSpeed - directedTurn);
  applyMotorSpeeds(desiredLeft, desiredRight);
}

// =================================================================================================
// Sensor-driven line-loss safety. There is no reverse motion or stored-action recovery.

void beginReacquireVerification() {
  stopMotors();
  consecutiveReacquiredSamples = 1;
  setControlState(STATE_REACQUIRE_VERIFY);
}

void completeLineReacquisition() {
  stopMotors();
  resetPidState();
  consecutiveLostSamples = 0;
  consecutiveReacquiredSamples = 0;
  setControlState(STATE_TRACKING);
}

void updateControlState(float dtSeconds) {
  switch (controlState) {
    case STATE_TRACKING:
      if (!blackDetected) {
        consecutiveLostSamples = 1;
        stopMotors();
        resetPidState();
        setControlState(STATE_LOSS_PENDING);
      } else {
        consecutiveLostSamples = 0;
        driveWithPidAndConfidence(dtSeconds);
      }
      break;

    case STATE_LOSS_PENDING:
      stopMotors();
      if (blackDetected) {
        beginReacquireVerification();
      } else {
        consecutiveLostSamples++;
        if (consecutiveLostSamples >= LINE_LOST_CONFIRMATION_SAMPLES) {
          setControlState(STATE_LINE_LOST);
        }
      }
      break;

    case STATE_LINE_LOST:
      stopMotors();
      if (blackDetected) beginReacquireVerification();
      break;

    case STATE_REACQUIRE_VERIFY:
      stopMotors();
      if (blackDetected) {
        consecutiveReacquiredSamples++;
        if (consecutiveReacquiredSamples >= LINE_REACQUIRE_CONFIRMATION_SAMPLES) {
          completeLineReacquisition();
        }
      } else {
        consecutiveReacquiredSamples = 0;
        consecutiveLostSamples = LINE_LOST_CONFIRMATION_SAMPLES;
        setControlState(STATE_LINE_LOST);
      }
      break;
  }
}

// =================================================================================================
// Telemetry and optional scope output.

bool scopeDebugConfigured() {
  return ENABLE_SCOPE_DEBUG
      && SCOPE_ERROR_PIN >= 0
      && SCOPE_TURN_PIN >= 0
      && SCOPE_SYNC_PIN >= 0;
}

int signedValueToScopeDuty(float value, float fullScale) {
  float bounded = clampFloat(value, -fullScale, fullScale);
  float fraction = (bounded + fullScale) / (2.0f * fullScale);
  return clampInt((int)roundf(255.0f * fraction), 0, 255);
}

void writeScopeDebug() {
  if (!scopeDebugConfigured()) return;
  ledcWrite(SCOPE_ERROR_PIN, signedValueToScopeDuty(lineError, 3.0f));
  ledcWrite(SCOPE_TURN_PIN, signedValueToScopeDuty(appliedTurn, MAX_TURN));
}

void printTelemetry(unsigned long nowMs) {
  if (!ENABLE_SERIAL_TELEMETRY || nowMs - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = nowMs;

  // Arduino Serial Plotter accepts label:value fields separated by tabs.
  Serial.print("error:");
  Serial.print(lineError, 4);
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
  Serial.print(appliedTurn, 3);
  Serial.print('\t');
  Serial.print("confidence:");
  Serial.print(lineConfidence, 3);
  Serial.print('\t');
  Serial.print("checksPassed:");
  Serial.print(confidenceAccumulator.checksPassed);
  Serial.print('\t');
  Serial.print("checksCompleted:");
  Serial.print(confidenceAccumulator.checksCompleted);
  Serial.print('\t');
  Serial.print("pidChecks:");
  Serial.print(consecutivePidChecks);
  Serial.print('\t');
  Serial.print("black:");
  Serial.print(blackDetected ? 1 : 0);
  Serial.print('\t');
  Serial.print("leftCmd:");
  Serial.print(leftMotorSpeed);
  Serial.print('\t');
  Serial.print("rightCmd:");
  Serial.print(rightMotorSpeed);
  Serial.print('\t');
  Serial.print("leftPwm:");
  Serial.print(leftAppliedPwm);
  Serial.print('\t');
  Serial.print("rightPwm:");
  Serial.print(rightAppliedPwm);
  Serial.print('\t');
  Serial.print("Kp:");
  Serial.print(kP, 3);
  Serial.print('\t');
  Serial.print("Ki:");
  Serial.print(kI, 3);
  Serial.print('\t');
  Serial.print("Kd:");
  Serial.print(kD, 3);
  Serial.print('\t');
  Serial.print("saturated:");
  Serial.print(outputWasSaturated ? 1 : 0);
  Serial.print('\t');
  Serial.print("state:");
  Serial.println((int)controlState);
}

void printHelp() {
  Serial.println("Commands: h = help");
  Serial.print("State: ");
  Serial.println(stateName(controlState));
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = (char)Serial.read();
    if (command == 'h' || command == 'H') printHelp();
  }
}

// =================================================================================================
// Arduino entry points.

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  ledcAttach(M1H, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttach(M1L, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttach(M2H, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttach(M2L, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION_BITS);

  for (int led = 0; led < LED_COUNT; led++) pinMode(LED_PINS[led], OUTPUT);

  if (scopeDebugConfigured()) {
    ledcAttach(SCOPE_ERROR_PIN, SCOPE_PWM_FREQUENCY, SCOPE_PWM_RESOLUTION_BITS);
    ledcAttach(SCOPE_TURN_PIN, SCOPE_PWM_FREQUENCY, SCOPE_PWM_RESOLUTION_BITS);
    pinMode(SCOPE_SYNC_PIN, OUTPUT);
    digitalWrite(SCOPE_SYNC_PIN, LOW);
  }

  stopMotors();
  calibrateSensors();
  readPotentiometers();
  resetPidState();

  unsigned long nowMs = millis();
  lastControlMs = nowMs;
  lastTelemetryMs = nowMs;
  setControlState(STATE_TRACKING);
  printHelp();
}

void loop() {
  handleSerialCommands();

  unsigned long nowMs = millis();
  if (nowMs - lastControlMs < CONTROL_INTERVAL_MS) return;

  float dtSeconds = (float)(nowMs - lastControlMs) / 1000.0f;
  dtSeconds = clampFloat(dtSeconds, 0.001f, 0.050f);
  lastControlMs = nowMs;

  if (scopeDebugConfigured()) digitalWrite(SCOPE_SYNC_PIN, HIGH);

  readPotentiometers();
  readPhotoresistors();
  calculateLineErrorAndSensorConfidence();
  updateControlState(dtSeconds);
  writeScopeDebug();
  if (scopeDebugConfigured()) digitalWrite(SCOPE_SYNC_PIN, LOW);
  printTelemetry(nowMs);
}

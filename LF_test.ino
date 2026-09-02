/* ************************************************************************************************* /
// UCSD ECE 5 Lab 4 Code: Line Following Robot with PID
// V 5.0
// Last Modified 8/24/2026 by MingWei Yeoh, Karcher Morris, and Korey Huynh
/ ************************************************************************************************* */

/*
This is code for your PID controlled line following robot.

******      Code Table of Contents      ******

Line_Follower_Code_Basic

Declare libraries     - declares global variables so each variable can be accessed from every function
Declare Pins          - where the user sets what pin everything is connected to
Settings              - settings that can improve robot functionality and help to debug
Setup (Main)          - runs once at beginning when you press button on arduino or when you open serial monitor
Loop  (Main)          - loops forever calling on a series of function

Calibration

Main Calibrate()      - runs calibration function calls and synchronizes calibration state with different led animations

Helper_Functions

setLEDs               - turns on all LEDs in the LED_Pin array on or off
Read Potentiometers   - reads each potentiometer
Read Photoresistors   - reads each photoresistor
Run Motors            - runs motors
Calculate Error       - calculate error from photoresistor readings
PID Turn              - takes the error and implements PID control
Print                 - used for printing information but should disable when not debugging because it slows down program

*/

// Include files needed
//#include <L298NX2.h> // Using "L298N" library found through arduino library manager developed by Andrea Lombardo (https://github.com/AndreaLombardo/L298N)

// ************************************************************************************************* //
// ************************************************************************************************* //
// Change Robot Settings here

#define PRINTALLDATA       0// Human-readable diagnostics; disable while driving
#define PLOTTERDATA        1// Compact numeric telemetry for Arduino Serial Plotter
#define SCOPEDEBUG         0// Optional PWM copies of error and turn on pins 12/13
// !! Turn to 0 when running robot untethered
#define NOMINALSPEED        110// This is the base speed for both motors, can also be increased by using potentiometers

// Control and detection settings. Tune these after looking at calibration/plotter data.
const unsigned long CONTROL_PERIOD_US = 10000; // 100 Hz controller
const float MIN_LINE_CONFIDENCE = 0.60;
const float LOW_CONFIDENCE_SPEED_SCALE = 0.35;
const float FULL_CONFIDENCE_SPEED_SCALE = 1.00;
const int MAX_REQUESTED_CRUISE_SPEED = 220;
const int MAX_TURN = 160;
const float MAX_TURN_CHANGE_PER_SECOND = 1200.0;
const float DERIVATIVE_FILTER_TIME_CONSTANT_SECONDS = 0.040;
const float INTEGRAL_LIMIT = 3.0;
const int LOST_LINE_CONFIRM_CYCLES = 3;
const int REACQUIRE_CONFIRM_CYCLES = 2;
const int ACTION_HISTORY_SIZE = 16;
const unsigned long ACTION_INTERVAL_MS = 20;
const unsigned long TRACEBACK_SETTLE_MS = 60;
const int TRACEBACK_MAX_PWM = 160;
const float TRACEBACK_REVERSE_SLEW_PWM_PER_SECOND = 4000.0;
const unsigned long TELEMETRY_INTERVAL_MS = 50;
const int SCOPE_ERROR_PIN = 12;
const int SCOPE_TURN_PIN = 13;

// ************************************************************************************************* //

// ****** DECLARE PINS HERE  ******

// Taken from LEFT TO RIGHT of the robot ****** Orient yourself so that you are looking from the rear of the robot (photoresistors are farthest away from you, wheels are closest to you)

enum side {LEFT, RIGHT};

int LDR_Pin[] = {1, 2, 3, 4, 5, 6, 7}; // SET PINS CONNECTED TO PHOTORESISTORS // FROM LEFT TO RIGHT OF THE ROBOT, ROBOT IS ORIENTED WHERE PHOTORESISOTRS FARTHEST FROM YOU AND WHEELS ARE CLOSEST TO YOU

int M1H = 45;
int M1L = 46;
int M2H = 43;
int M2L = 44;

// Potentiometer Pins
const int S_pin = 10; // Pin connected to Speed potentiometer
const int P_pin = 11; // Pin connected to P term potentiometer
const int I_pin = 8; // Pin connected to I term potentiometer
const int D_pin = 9; // Pin connected to D term potentiometer

int led_Pins[] = {17};  // LEDs to indicate what part of calibration you're on and to illuminate the photoresistors

// ****** DECLARE Variables HERE  ******

//Variables Potentiometer Reading
int SpRead = 0; // speed increase
int kPRead = 0; // proportional gain
int kIRead = 0; // integral gain
int kDRead = 0; // derivative gain

// Variables for Calibration and Error Calculation
float Mn[20];
float Mx[20];
float LDRf[20];
int LDR[20];
int rawPResistorData[20];
int totalPhotoResistors = sizeof(LDR_Pin) / sizeof(LDR_Pin[0]);
int numLEDs = sizeof(led_Pins) / sizeof(led_Pins[0]);
int MxRead, MxIndex;
float CriteriaForMax;
int leftHighestPR, highestPResistor, rightHighestPR;
float AveRead, WeightedAve;

// For Motor Control
int M1SpeedtoMotor, M2SpeedtoMotor;
float Turn = 0.0;
int M1P = 0, M2P = 0;
float error, lasterror = 0, sumerror = 0;
float kP, kI, kD;
float lineConfidence = 0.0;
float sensorConfidence = 0.0;
float calibrationConfidence = 1.0;
float derivativeFiltered = 0.0;
float previousTurn = 0.0;
bool lineDetected = false;
bool tracingBack = false;
int lostLineCycles = 0;
int reacquireCycles = 0;
unsigned long previousControlTime = 0;
unsigned long previousActionTime = 0;
unsigned long previousTelemetryTime = 0;
unsigned long tracebackStartedAt = 0;
float previousErrorRate = 0.0;
float lastTraceLeft = 0.0;
float lastTraceRight = 0.0;
bool pidStableHistory[6] = {true, true, true, true, true, true};
int pidHistoryIndex = 0;

struct WheelAction {
int leftSpeed;
int rightSpeed;
};

WheelAction actionHistory[ACTION_HISTORY_SIZE];
int historyHead = 0;
int historyCount = 0;

String whitevals = "White: ";

String blackvals = "Black: ";

String deltavals = "Delta: ";

// ************************************************************************************************* //
// setup - runs once

int reading = 0;

void setup() {
Serial.begin(115200);                          // Fast enough for nonblocking plotter telemetry
ledcAttach(M1H, 12000, 8);  // 12 kHz PWM, 8-bit resolution
ledcAttach(M1L, 12000, 8);
ledcAttach(M2H, 12000, 8);
ledcAttach(M2L, 12000, 8);
if (SCOPEDEBUG) {
ledcAttach(SCOPE_ERROR_PIN, 12000, 8);
ledcAttach(SCOPE_TURN_PIN, 12000, 8);
}
for (int i = 0; i < numLEDs; i++)
pinMode(led_Pins[i], OUTPUT);                // Initialize all LEDs to output

Calibrate();                                   // Calibrate black and white sensing
float spanQualitySum = 0.0;
for (int i = 0; i < totalPhotoResistors; i++)
spanQualitySum += constrain(abs(Mx[i] - Mn[i]) / 1000.0, 0.0, 1.0);
calibrationConfidence = spanQualitySum / totalPhotoResistors;
for (int i = 0; i < totalPhotoResistors; i++)
whitevals += String(Mn[i]) + " ";  // Store the White values that will be used by the robot

for (int i = 0; i < totalPhotoResistors; i++)
  blackvals += String(Mx[i]) + " ";  // Store the Black values that will be used by the robot

for (int i = 0; i < totalPhotoResistors; i++)
  deltavals += String(Mx[i] - Mn[i]) + " ";  // Print the Difference between the White and Black valuess

ReadPotentiometers();                          // Read potentiometer values (Sp, P, I, & D)
previousControlTime = micros();

} // end setup()



// ************************************************************************************************* //
// loop - runs/loops forever
void loop() {
unsigned long now = micros();
if ((unsigned long)(now - previousControlTime) < CONTROL_PERIOD_US)
return;

float dt = (float)(now - previousControlTime) / 1000000.0;
previousControlTime = now;

ReadPotentiometers(); // Read potentiometers

ReadPhotoResistors(); // Read photoresistors

CalcError();          // Calculates error

if (sensorConfidence < MIN_LINE_CONFIDENCE) {
lostLineCycles++;
reacquireCycles = 0;
} else {
reacquireCycles++;
if (reacquireCycles >= REACQUIRE_CONFIRM_CYCLES) {
lineDetected = true;
lostLineCycles = 0;
tracingBack = false;
}
}

if (lostLineCycles >= LOST_LINE_CONFIRM_CYCLES) {
if (!tracingBack)
tracebackStartedAt = millis();
tracingBack = true;
if (historyCount > 0) {
if (millis() - tracebackStartedAt < TRACEBACK_SETTLE_MS) {
runMotorAtSpeed(LEFT, 0);
runMotorAtSpeed(RIGHT, 0);
} else {
TraceBack(dt);   // Physically undo recent wheel actions until the line is found
}
} else {
tracingBack = true;
M1SpeedtoMotor = 0;
M2SpeedtoMotor = 0;
runMotorAtSpeed(LEFT, 0);
runMotorAtSpeed(RIGHT, 0);
}
} else {
PID_Turn(dt);        // Fixed-time PID control
RunMotors();         // Confidence-aware motor mixing
}

if (PRINTALLDATA)     // If PRINTALLDATA Enabled, Print all the data
Print();
if (PLOTTERDATA && millis() - previousTelemetryTime >= TELEMETRY_INTERVAL_MS) {
PrintPlotter();
previousTelemetryTime = millis();
}
} // end loop()





// ************************************************************************************************* //
// function to calibrate

void Calibrate() {

int numberOfMeasurements = 50;                // set number Of Measurements to take

CalibrateHelper(numberOfMeasurements, false); // White Calibration

setLeds(0);                                   // Turn off LEDs to indicate user to calibrate other color
delay(2000);

CalibrateHelper(numberOfMeasurements, true);  // Black Calibration

Serial.print("White Vals:  ");
for (int i = 0; i < totalPhotoResistors; i++)
Serial.print(String(Mn[i]) + " ");          // Print the White values that will be used by the robot
Serial.println();

Serial.print("Black Vals:  ");
for (int i = 0; i < totalPhotoResistors; i++)
Serial.print(String(Mx[i]) + " ");          // Print the Black values that will be used by the robot
Serial.println();

Serial.print("Delta Vals:  ");
for (int i = 0; i < totalPhotoResistors; i++)
Serial.print(String(Mx[i] - Mn[i]) + " ");  // Print the Difference between the White and Black valuess
Serial.println();

setLeds(1);                                   // Turn LEDs on
delay(2000);

} // end Calibrate()

void CalibrateHelper(int numberOfMeasurements, boolean ifCalibratingBlack) {

if (ifCalibratingBlack)
Serial.println("\nCalibrating Black");
else
Serial.println("\nCalibrating White");
// Indicate that calibration is starting
for (int i = 0; i < 4; i++) {
setLeds(1); // turn the LEDs on
delay(250); // wait
setLeds(0); // turn the LEDs off
delay(250); // wait
}

setLeds(1);
delay(250);

for (int i = 0; i < numberOfMeasurements; i++) {
for (int pin = 0; pin < totalPhotoResistors; pin++) {
LDRf[pin] = LDRf[pin] + (float)analogRead(LDR_Pin[pin]);
delay(2);
}
Serial.print(". ");
}
for (int pin = 0; pin < totalPhotoResistors; pin++) {
if (ifCalibratingBlack) {                                   // updating cooresponding array based on if we are calibrating black or white
Mx[pin] = round(LDRf[pin] / (float)numberOfMeasurements); // take average and store for black
}
else {
Mn[pin] = round(LDRf[pin] / (float)numberOfMeasurements); // take average and store for white
}
LDRf[pin] = 0.0;
}

Serial.println(" Done!");
setLeds(0);
delay(250);
}

// Set all LEDs to a certain brightness
void setLeds(int x) {
for (int i = 0; i < numLEDs; i++)
digitalWrite(led_Pins[i], x);
}

// Recall your Challenge #1 Code************************************************************ //
// function to read and map values from potentiometers
void ReadPotentiometers() {
// Call on user-defined function to read Potentiometer values
SpRead = ReadPotentiometerHelper(S_pin, 0, 4095, 0, 100); // We want to read a potentiometer for S_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.
kPRead = ReadPotentiometerHelper(P_pin, 0, 4095, 0, 100); // We want to read a potentiometer for P_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.
kIRead = ReadPotentiometerHelper(I_pin, 0, 4095, 0, 100); // We want to read a potentiometer for I_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.
kDRead = ReadPotentiometerHelper(D_pin, 0, 4095, 0, 100); // We want to read a potentiometer for D_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.

} // end ReadPotentiometers()

int ReadPotentiometerHelper(int pin, int min_resolution, int max_resolution, int min_potentiometer, int max_potentiometer) {
return map(analogRead(pin), min_resolution, max_resolution, min_potentiometer, max_potentiometer);
}

// Recall your Challenge #2 Code************************************************************ //
// Function to read photo resistors and map from 0 to 100
void ReadPhotoResistors() {
for (int i = 0; i < totalPhotoResistors; i++) {
rawPResistorData[i] = analogRead(LDR_Pin[i]);
float span = Mx[i] - Mn[i];
if (abs(span) < 1.0) {
LDR[i] = 0; // invalid calibration span; avoid division by zero
} else {
float normalized = 100.0 * ((float)rawPResistorData[i] - Mn[i]) / span;
LDR[i] = constrain((int)round(normalized), 0, 100);
}
}

} // end ReadPhotoResistors()

// Recall your Challenge #3 Code************************************************************ //
// function to start motors using nominal speed + speed addition from potentiometer
void RunMotors() {
float confidenceScale = LOW_CONFIDENCE_SPEED_SCALE +
constrain(lineConfidence, 0.0f, 1.0f) *
(FULL_CONFIDENCE_SPEED_SCALE - LOW_CONFIDENCE_SPEED_SCALE);
float requestedBase = min((float)(NOMINALSPEED + SpRead),
(float)MAX_REQUESTED_CRUISE_SPEED);
float baseSpeed = requestedBase * confidenceScale;

float leftCommand = baseSpeed - Turn;
float rightCommand = baseSpeed + Turn;

// Preserve the left/right ratio if either command exceeds the PWM range.
float largest = max(abs(leftCommand), abs(rightCommand));
if (largest > 255.0) {
float scale = 255.0 / largest;
leftCommand *= scale;
rightCommand *= scale;
}

M1SpeedtoMotor = constrain((int)round(leftCommand), -255, 255);
M2SpeedtoMotor = constrain((int)round(rightCommand), -255, 255);

runMotorAtSpeed(LEFT, M2SpeedtoMotor); // run right motor
runMotorAtSpeed(RIGHT, M1SpeedtoMotor); // run left motor
// Preserve the actual logical LEFT/RIGHT commands, matching the calls above.
StoreAction(M2SpeedtoMotor, M1SpeedtoMotor);
} // end RunMotors()

void StoreAction(int leftSpeed, int rightSpeed) {
if (millis() - previousActionTime < ACTION_INTERVAL_MS)
return;
previousActionTime = millis();
actionHistory[historyHead] = {leftSpeed, rightSpeed};
historyHead = (historyHead + 1) % ACTION_HISTORY_SIZE;
if (historyCount < ACTION_HISTORY_SIZE)
historyCount++;
}

void TraceBack(float dt) {
tracingBack = true;
historyHead = (historyHead - 1 + ACTION_HISTORY_SIZE) % ACTION_HISTORY_SIZE;
WheelAction action = actionHistory[historyHead];
historyCount--;

float targetLeft = constrain(-action.leftSpeed, -TRACEBACK_MAX_PWM, TRACEBACK_MAX_PWM);
float targetRight = constrain(-action.rightSpeed, -TRACEBACK_MAX_PWM, TRACEBACK_MAX_PWM);
float maxChange = TRACEBACK_REVERSE_SLEW_PWM_PER_SECOND * dt;
lastTraceLeft = constrain(targetLeft, lastTraceLeft - maxChange, lastTraceLeft + maxChange);
lastTraceRight = constrain(targetRight, lastTraceRight - maxChange, lastTraceRight + maxChange);
int reverseLeft = (int)round(lastTraceLeft);
int reverseRight = (int)round(lastTraceRight);
M2SpeedtoMotor = reverseLeft;
M1SpeedtoMotor = reverseRight;
runMotorAtSpeed(LEFT, reverseLeft);
runMotorAtSpeed(RIGHT, reverseRight);
}

// A function that commands a specified motor to move towards a given direction at a given speed
void runMotorAtSpeed(side _side, int speed) {
if (_side == LEFT) {
if (speed > 0)         {       // swap direction if speed is negative
ledcWrite(M1H, speed);
ledcWrite(M1L, 0);          // sets the direction of the motor from arguments
} else {
ledcWrite(M1H, 0);
ledcWrite(M1L, abs(speed)); // sets the direction of the motor from arguments
}
}
if (_side == RIGHT) {
if (speed > 0)  {              // swap direction if speed is negative
ledcWrite(M2H, speed);
ledcWrite(M2L, 0); // sets the direction of the motor from arguments
} else {
ledcWrite(M2H, 0);
ledcWrite(M2L, abs(speed)); // sets the direction of the motor from arguments
}
}
}



// ************************************************************************************************* //
// Calculate error from photoresistor readings
void CalcError() {
float weightedSum = 0.0;
float sensorSum = 0.0;
float peak = 0.0;
float minimum = 100.0;
int activeSensors = 0;
int peakIndex = 0;

for (int i = 0; i < totalPhotoResistors; i++) {
float position = (float)i - ((float)(totalPhotoResistors - 1) / 2.0);
weightedSum += (float)LDR[i] * position;
sensorSum += (float)LDR[i];
if (LDR[i] > peak) {
peak = LDR[i];
peakIndex = i;
}
minimum = min(minimum, (float)LDR[i]);
if (LDR[i] >= 25)
activeSensors++;
}

float average = sensorSum / (100.0 * totalPhotoResistors);
float peakScore = peak / 100.0;
float contrastScore = (peak - minimum) / 100.0;
float activeScore = constrain(1.0 - abs(activeSensors - 2) / 5.0, 0.0, 1.0);
float localEnergy = LDR[peakIndex];
if (peakIndex > 0) localEnergy += LDR[peakIndex - 1];
if (peakIndex + 1 < totalPhotoResistors) localEnergy += LDR[peakIndex + 1];
float localScore = sensorSum > 0.0 ? constrain(localEnergy / sensorSum, 0.0, 1.0) : 0.0;

// Sensor-side portion of yesterday's eight-factor confidence score.
sensorConfidence = 0.10 * calibrationConfidence +
0.20 * peakScore +
0.15 * contrastScore +
0.10 * activeScore +
0.10 * localScore;
sensorConfidence /= 0.65; // normalize sensor-only evidence to 0..1 for loss detection
sensorConfidence = constrain(sensorConfidence, 0.0, 1.0);
lineDetected = sensorConfidence >= MIN_LINE_CONFIDENCE;

// Keep the last meaningful error during a brief loss; traceback handles a confirmed loss.
if (sensorSum > 0.0)
error = weightedSum / sensorSum;

} // end CalcError()

// ************************************************************************************************* //
// PID Function
void PID_Turn(float dt) {
kP = (float)kPRead * 1.;    // each of these scaling factors can change depending on how influential you want them to be
kI = (float)kIRead * 0.001;
kD = (float)kDRead * 0.02;

if (dt <= 0.0)
dt = (float)CONTROL_PERIOD_US / 1000000.0;

float derivativeRaw = (error - lasterror) / dt;
float derivativeAlpha = dt / (DERIVATIVE_FILTER_TIME_CONSTANT_SECONDS + dt);
derivativeFiltered += derivativeAlpha * (derivativeRaw - derivativeFiltered);

float candidateIntegral = constrain(sumerror + error * dt,
-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
float unsaturatedTurn = error * kP + candidateIntegral * kI + derivativeFiltered * kD;

float continuityScore = 1.0 - constrain(abs(error - lasterror) / 3.0, 0.0, 1.0);
float pidOutputScore = 1.0 - constrain(abs(unsaturatedTurn) / 220.0, 0.0, 1.0);
bool pidStable = abs(derivativeRaw) <= 150.0 && abs(unsaturatedTurn) <= 220.0;
pidStableHistory[pidHistoryIndex] = pidStable;
pidHistoryIndex = (pidHistoryIndex + 1) % 6;
int stableChecks = 0;
for (int i = 0; i < 6; i++)
stableChecks += pidStableHistory[i] ? 1 : 0;
float historyScore = stableChecks / 6.0;
lineConfidence = constrain(0.65 * sensorConfidence +
0.10 * continuityScore +
0.10 * pidOutputScore +
0.15 * historyScore,
0.0, 1.0);

// Conditional integration: do not wind the integrator farther into saturation.
bool drivesOutOfSaturation = (unsaturatedTurn > MAX_TURN && error < 0.0) ||
(unsaturatedTurn < -MAX_TURN && error > 0.0);
if (abs(unsaturatedTurn) <= MAX_TURN || drivesOutOfSaturation)
sumerror = candidateIntegral;

if (abs(error) < 0.02)
sumerror = 0.0;

float requestedTurn = error * kP + sumerror * kI + derivativeFiltered * kD;
requestedTurn = constrain(requestedTurn, -(float)MAX_TURN, (float)MAX_TURN);
float maxTurnChange = MAX_TURN_CHANGE_PER_SECOND * dt;
Turn = constrain(requestedTurn,
previousTurn - maxTurnChange,
previousTurn + maxTurnChange);
previousTurn = Turn;

lasterror = error;

if (SCOPEDEBUG) {
ledcWrite(SCOPE_ERROR_PIN, constrain((int)round((error + 3.0) * 255.0 / 6.0), 0, 255));
ledcWrite(SCOPE_TURN_PIN, constrain((int)round((Turn + MAX_TURN) * 255.0 /
(2.0 * MAX_TURN)), 0, 255));
}

} // end PID_Turn()

// ************************************************************************************************* //
// function to print values of interest
void Print() {
Serial.print(" Sp: " + String(SpRead) + " P: " + String(kP) + " I: " + String(kI) + " D: " + String(kD) + "  PResistor Val : "); // Prints PID settings

for (int i = 0; i < totalPhotoResistors; i++) { // Printing the photo resistor reading values one by one
Serial.print(LDR[i]);
//Serial.print(rawPResistorData[i]); //Uncomment this if you would prefer to see raw photoresistor readings
Serial.print(" ");
}

Serial.print(" Error: " + String(error));      // this will show the calculated error (-3 through 3)

Serial.println("  LMotor:  " + String(M1SpeedtoMotor) + "  RMotor:  " + String(M2SpeedtoMotor));    // This prints the arduino output to each motor so you can see what the values are (0-255)
setLeds(0);
delay(100);                                    // just here to slow down the output for easier reading. Don't comment out or else it'll slow down the processor on the arduino
setLeds(1);
delay(100);

} // end Print()

void PrintPlotter() {
Serial.print("Error:");
Serial.print(error, 3);
Serial.print(" Confidence:");
Serial.print(lineConfidence, 1);
Serial.print(" Turn:");
Serial.print(Turn, 1);
Serial.print(" LeftMotor:");
Serial.print(M1SpeedtoMotor);
Serial.print(" RightMotor:");
Serial.print(M2SpeedtoMotor);
Serial.print(" LineDetected:");
Serial.print(lineDetected ? 100 : 0);
Serial.print(" Traceback:");
Serial.println(tracingBack ? 100 : 0);
}

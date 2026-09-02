/* ************************************************************************************************* */
// UCSD ECE 5 Lab 4 Code: Line Following Robot with PID 
// V 5.0
// Last Modified 8/24/2026 by MingWei Yeoh, Karcher Morris, and Korey Huynh
/* ************************************************************************************************* */


/*
   This is code for your PID controlled line following robot.

   ******      Code Table of Contents      ******

  - Line_Follower_Code_Basic
   > Declare libraries     - declares global variables so each variable can be accessed from every function
   > Declare Pins          - where the user sets what pin everything is connected to 
   > Settings              - settings that can improve robot functionality and help to debug
   > Setup (Main)          - runs once at beginning when you press button on arduino or when you open serial monitor
   > Loop  (Main)          - loops forever calling on a series of function
   
  - Calibration 
   > Main Calibrate()      - runs calibration function calls and synchronizes calibration state with different led animations
  
  - Helper_Functions
   > setLEDs               - turns on all LEDs in the LED_Pin array on or off
   > Read Potentiometers   - reads each potentiometer
   > Read Photoresistors   - reads each photoresistor
   > Run Motors            - runs motors
   > Calculate Error       - calculate error from photoresistor readings
   > PID Turn              - takes the error and implements PID control
   > Print                 - used for printing information but should disable when not debugging because it slows down program

*/

// Include files needed
//#include <L298NX2.h> // Using "L298N" library found through arduino library manager developed by Andrea Lombardo (https://github.com/AndreaLombardo/L298N)

// ************************************************************************************************* //
// ************************************************************************************************* //
// Change Robot Settings here

#define PRINTALLDATA       0// Turn to 1  to prints ALL the data when changed to 1, Could be useful for debugging =)
                                // !! Turn to 0 when running robot untethered
#define NOMINALSPEED        255// This is the base speed for both motors, can also be increased by using potentiometers

// ************************************************************************************************* //

// ****** DECLARE PINS HERE  ****** 

// Taken from LEFT TO RIGHT of the robot ****** Orient yourself so that you are looking from the rear of the robot (photoresistors are farthest away from you, wheels are closest to you)

enum side {LEFT, RIGHT};

//Photoresistor variables
int LDR_Pin[] = {1, 2, 3, 4, 5, 6, 7}; // SET PINS CONNECTED TO PHOTORESISTORS // FROM LEFT TO RIGHT OF THE ROBOT, ROBOT IS ORIENTED WHERE PHOTORESISOTRS FARTHEST FROM YOU AND WHEELS ARE CLOSEST TO YOU      
float filteredLDR[20] = {0};
//Lower = more filtering, higher = faster response
const float ALPHA = 1;
bool filterInitialized = false;

//
int M1H = 45;
int M1L = 46;
int M2H = 43;
int M2L = 44;

// Potentiometer Pins
const int S_pin = 10; // Pin connected to Speed potentiometer
const int P_pin = 11; // Pin connected to P term potentiometer
const int I_pin = 14; // Pin connected to I term potentiometer
const int D_pin = 13; // Pin connected to D term potentiometer
                                                                 
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
int MxRead, MxIndex, CriteriaForMax;
int leftHighestPR, highestPResistor, rightHighestPR;
float AveRead, WeightedAve;   

// For Motor Control
const int speedControl = 15;
int max_speed = 255;
int speed = 0;

int M1SpeedtoMotor, M2SpeedtoMotor;
int Turn, M1P = 0, M2P = 0;
float error, lasterror = 0, sumerror = 0;
float kP, kI, kD;


String whitevals = "White: ";

String blackvals = "Black: ";


String deltavals = "Delta: ";


// ************************************************************************************************* //
// setup - runs once

int reading = 0;

void setup() {
  Serial.begin(9600);                            // For serial communication set up
  ledcAttach(M1H, 12000, 8);  // 12 kHz PWM, 8-bit resolution
  ledcAttach(M1L, 12000, 8);
  ledcAttach(M2H, 12000, 8);
  ledcAttach(M2L, 12000, 8);
  for (int i = 0; i < numLEDs; i++)
    pinMode(led_Pins[i], OUTPUT);                // Initialize all LEDs to output
  
  Calibrate();                                   // Calibrate black and white sensing
      for (int i = 0; i < totalPhotoResistors; i++)
      whitevals += String(Mn[i]) + " ";  // Store the White values that will be used by the robot
    
    for (int i = 0; i < totalPhotoResistors; i++)
      blackvals += String(Mx[i]) + " ";  // Store the Black values that will be used by the robot
  
    for (int i = 0; i < totalPhotoResistors; i++)
      deltavals += String(Mx[i] - Mn[i]) + " ";  // Print the Difference between the White and Black valuess

    ReadPotentiometers();                          // Read potentiometer values (Sp, P, I, & D)
} // end setup()



// ************************************************************************************************* //
// loop - runs/loops forever
void loop() {
  ReadPotentiometers(); // Read potentiometers

  ReadPhotoResistors(); // Read photoresistors 

  CalcError();          // Calculates error

  PID_Turn();           // PID Control and Output to motors to turn

  RunMotors();          // Runs motors
  
  if (PRINTALLDATA)     // If PRINTALLDATA Enabled, Print all the data
    Print();         
} // end loop()






// ************************************************************************************************* //
// function to calibrate

void Calibrate() {

  int numberOfMeasurements = 20;                // set number Of Measurements to take

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

// **********Recall your Challenge #1 Code********************************************************************** //
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

// **********Recall your Challenge #2 Code********************************************************************** //
// Function to read photo resistors and map from 0 to 100
void ReadPhotoResistors() {

  for (int i = 0; i < totalPhotoResistors; i++) {

    rawPResistorData[i] = analogRead(LDR_Pin[i]);

    if (!filterInitialized) {
      filteredLDR[i] = rawPResistorData[i];
    }
    else {
      filteredLDR[i] = ALPHA * rawPResistorData[i]
                     + (1.0 - ALPHA) * filteredLDR[i];
    }

    LDR[i] = 100.0 *
             (filteredLDR[i] - Mn[i]) /
             (Mx[i] - Mn[i]);

    LDR[i] = constrain(LDR[i], 0, 100);
  }

  filterInitialized = true;

} // end ReadPhotoResistors()


// **********Recall your Challenge #3 Code********************************************************************** //
// function to start motors using nominal speed + speed addition from potentiometer
void RunMotors() {
  M1SpeedtoMotor = min(max(NOMINALSPEED + SpRead + M1P,-speed), speed); 
  M2SpeedtoMotor = min(max(NOMINALSPEED + SpRead + M2P,-speed), speed); // remember M1Sp & M2Sp is defined at beginning of code (default 60)
  
  runMotorAtSpeed(LEFT, M2SpeedtoMotor); // run right motor 
  runMotorAtSpeed(RIGHT, M1SpeedtoMotor); // run left motor
} // end RunMotors()

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
  MxRead = -99;
  AveRead = 0.0;
  for (int i = 0; i < totalPhotoResistors; i++) { // This loop goes through each photoresistor and stores the photoresistor with the highest value to the variable 'highestPResistor'
    if (MxRead < LDR[i]) {
      MxRead = LDR[i];
      MxIndex = -1 * (i - 3);
      highestPResistor = (float)i;
    }
    AveRead = AveRead + (float)LDR[i] / (float)totalPhotoResistors;
  }
  
  CriteriaForMax = 1.5; 
  if (MxRead > CriteriaForMax * AveRead) { // Make sure that the highestPResistor is actually "seeing" a line. What happens if there is no line and we take the photoresistor that happens to have the highest value?

    // Next we assign variables to hold the index of the left and right Photoresistor that has the highest value, though we have to make sure that we aren't checking a Photoresistor that doesn't exist.
    // Ex: To the left of the left most photoresistor or the right of the right most photoresistor
    if (highestPResistor != 0)
      leftHighestPR = highestPResistor - 1;
    else
      leftHighestPR = highestPResistor;

    if (highestPResistor != totalPhotoResistors - 1)
      rightHighestPR = highestPResistor + 1;
    else
      rightHighestPR = highestPResistor;

    // Next we take the percentage of "line" each of our left, middle, and right photoresistors sees and then we take the average, which is our error calculation
    float numerator = (float)(LDR[leftHighestPR] * leftHighestPR) + (float)(LDR[highestPResistor] * highestPResistor) + (float)(LDR[rightHighestPR] * rightHighestPR);
    float denominator = (float)LDR[leftHighestPR] + (float)LDR[highestPResistor] + (float)LDR[rightHighestPR];

    WeightedAve = ((float)numerator) / denominator;

    error = (WeightedAve - totalPhotoResistors/2);
  }
  speed = max_speed - speedControl * abs(error); //calculating value to clamp the max speed of robot to depending on how much error there is
} // end CalcError()

// ************************************************************************************************* //
// PID Function
void PID_Turn() {
  kP = (float)kPRead * 1.8;    // each of these scaling factors can change depending on how influential you want them to be
  kI = (float)kIRead * 0.001;
  kD = (float)kDRead * 0.01;

  Turn = error * kP + sumerror * kI + (error - lasterror) * kD; // PID!!!!!!!!!!!!!

  if (sumerror > 5)   // prevents integrator wind-up
    sumerror = 5;
  else if (sumerror < -5)
    sumerror = -5;

  if (error == 0)     // Reset sumerror if line is centered
    sumerror = 0;

  if (Turn < 0) {
    M1P = -Turn;       // One motor becomes slower and the other faster
    M2P = Turn;
  }
  else if (Turn > 0) {
    M1P = -Turn;
    M2P = Turn;
  }
  else {
    M1P = 0;
    M2P = 0;
  }

  lasterror = error;
  sumerror = sumerror + error;

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

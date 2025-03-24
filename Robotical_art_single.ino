/***************************************************
 *                Robotics_art_single.ino
 *
 *
 * 2D arm system used to draw images by moving end effector to predetermined locations.
 * Uses Geometric Inverse Kinemetics to determine joint angles as commands are already derivitive based
 * Uses commands such as penUp and penDown to determine if end effector is on page or not
 * Uses PID and PWM to determine voltage to motors.
 * This code takes assumes an initalization position where both arms of the robot are straight outwards.
Autors:
Cassidy Berzuk
Titus Waldner
 ***************************************************/

#include <Arduino.h>
#include <math.h>
#include "MeMegaPi.h"

// Arm lengths
static const float L1 = 11.0f; // Length of the first arm segment
static const float L2 = 6.5f;  // Length of the second arm segment

// Base angle constraints (to avoid the wall at x = 0)
static const float BASE_ANGLE_MIN = -65.0f; // Minimum base angle (degrees)
static const float BASE_ANGLE_MAX = 65.0f;  // Maximum base angle (degrees)

//penState

int penState = 1;

// Encoder and motor setup
static const byte BASE_ENC_A = 19;
static const byte BASE_ENC_B = 38;
static const byte ELBOW_ENC_A = 3;
static const byte ELBOW_ENC_B = 49;
static const byte PEN_ENC_A = 18;
static const byte PEN_ENC_B = 31;
volatile long baseEncoderCount = 0;
volatile long elbowEncoderCount = 0;
volatile long penEncoderCount = 0;
static const float BASE_COUNTS_PER_REV = 833.0f; // Encoder counts per revolution for base
static const float ELBOW_COUNTS_PER_REV = 833.0f; // Encoder counts per revolution for elbow
static const float PEN_COUNTS_PER_REV = 1398.0f;
MeMegaPiDCMotor motorPen(PORT1B);
MeMegaPiDCMotor motorBase(PORT2B);
MeMegaPiDCMotor motorElbow(PORT3B);

// PID control parameters
double baseKp = 2.5;    // Start low and increase gradually 2.5
double baseKi = 2.5;   // Slightly higher to reduce steady-state error
double baseKd = 0.2;    // Increase to dampen oscillations 0.2
double elbowKp = 2.5;   // Start low and increase gradually 2.5
double elbowKi = 2.5;  // Slightly higher to reduce steady-state error
double elbowKd = 0.2;   // Increase to dampen oscillations 0.2
static const float angleTolerance = 3.0f; // Tolerance for reaching target angles
static const double minEffort = 40.0f;    // Minimum motor effort
static const float MAX_ANGLE_STEP = 5.0f; // Maximum angle step per iteration

// Target coordinates
struct RobotCommand {
  float x;
  float y;
  int pen;
};
RobotCommand commands[] = {
  {15.0f, -5.0f, 1}, //up Left lip 
  //{15.0f, -4.0f, 0},
 /* {15.0f, -3.0f, 0},
  {15.0f, -2.0f, 0},
  {15.0f, -1.0f, 0},
  {15.0f, 0.0f, 0},
  {15.0f, 1.0f, 0},
  {15.0f, 2.0f, 0},
  {15.0f, 3.0f, 0},*/
  {15.0f, 0.0f, 0}, //down right lip
  {14.0f, -2.0f,1}, // up Left Eye
  {14.0f, -2.0f,0}, // down
  {14.0f, 0.0f,1}, //up Right Eye

  //{16.0f, 5.0f, 0},
  
};
const int NUM_COMMANDS = sizeof(commands) / sizeof(commands[0]);
int currentCommand = 0;

// Function prototypes
void setupTwoArmSystem();
void moveToXY(float xTarget, float yTarget);
static void computeIK(float x, float y, float &baseDeg, float &elbowDeg);
static void pidJointControl(float thetaBaseRef, float thetaElbowRef);
static float getBaseAngleDeg();
static float getElbowAngleDeg();
static void penEncISR();
static void baseEncISR();
static void elbowEncISR();
static void penDown();
static void penUp();


void setup() {
  penUp();
  Serial.begin(115200);
  setupTwoArmSystem();
  Serial.println("Main: Setup complete.");
}

void loop() {
  if (currentCommand < NUM_COMMANDS) {
    RobotCommand &cmd = commands[currentCommand];
    if(cmd.pen == 1)
    {
      penUp();
    }
    if(cmd.pen == 0)
    {
      penDown();
    }
  
    moveToXY(cmd.x, cmd.y);
    currentCommand++;
    delay(1000); // Pause between commands
  } else {
    penDown();
    Serial.println("All commands completed.");
    while (true) {}
  }
}

void setupTwoArmSystem() {
   pinMode(PEN_ENC_A, INPUT_PULLUP);
  pinMode(PEN_ENC_B, INPUT_PULLUP);
  pinMode(BASE_ENC_A, INPUT_PULLUP);
  pinMode(BASE_ENC_B, INPUT_PULLUP);
  pinMode(ELBOW_ENC_A, INPUT_PULLUP);
  pinMode(ELBOW_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PEN_ENC_A), penEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PEN_ENC_B), penEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BASE_ENC_A), baseEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BASE_ENC_B), baseEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELBOW_ENC_A), elbowEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELBOW_ENC_B), elbowEncISR, CHANGE);
  penEncoderCount = 0.0f; 
  baseEncoderCount = 0.0f;
  elbowEncoderCount = 0.0f; 
  Serial.println("TwoArmSystem with Ben: Setup complete.");
}

void penUp() {
  Serial.println("Starting Pen Up");
  Serial.println(penState);
  if(penState == 1)
  {
    Serial.println("Returning Early PenUp");
    return;
  }
  penState = 1;
  
  int angle = getPenAngleDeg();
  Serial.println("Pen Up: moving to target ");
 // Serial.println(target);
  
  while (angle < 55) {
    angle = getPenAngleDeg();
    Serial.println(angle);
    
    //error = target - getPenAngleDeg();
    
    motorPen.run((int) 80);
    delay(10);
  }
  
  motorPen.run(0);
  Serial.println("Pen Up reached.");
}

//------------------------------------------------------
// penDown() returns the pen to the baseline position.
// After reaching it, baseline is updated to account for any drift.
//------------------------------------------------------
void penDown() {
  Serial.println("Starting Pen Down");
  Serial.println(penState);
  if(penState == 0)
  {
    Serial.println("Returning Early PenDown");
    return;
  }
  penState = 0;
  int angle = getPenAngleDeg();
  Serial.println("Pen Down: moving to target ");
 // Serial.println(target);
  
  while (angle > 5) {
    angle = getPenAngleDeg();
    Serial.println(angle);
    
    //error = target - getPenAngleDeg();
    
    motorPen.run((int) -80);
    delay(10);
  }
  
  motorPen.run(0);
  Serial.println("Pen Down reached.");
}





void moveToXY(float xTarget, float yTarget) {
  float baseRefDeg, elbowRefDeg;
  computeIK(xTarget, yTarget, baseRefDeg, elbowRefDeg);
  Serial.print("Target: ("); Serial.print(xTarget); Serial.print(", "); Serial.print(yTarget);
  Serial.print(") -> baseRefDeg="); Serial.print(baseRefDeg); Serial.print(", elbowRefDeg="); Serial.println(elbowRefDeg);
  pidJointControl(baseRefDeg, elbowRefDeg);
}

/*
 * computeIK - Computes inverse kinematics for a 2-link planar robotic arm.
 * Given a target (x, y), it calculates the base and elbow joint angles (in degrees) 
 * using trigonometric laws (law of cosines and atan2). The function checks if the target 
 * is within the arm's reachable range, computes both "elbow up" and "elbow down" configurations, 
 * and selects one based on heuristic rules to avoid near-full extension and to favor positions 
 * above or below the base. If the target is unreachable, it retains the current joint angles.
 */

static void computeIK(float x, float y, float &baseDeg, float &elbowDeg) {
  float localX = x - 0.0f;
  float localY = y - 0.0f;
  float dist = sqrtf(localX * localX + localY * localY);
  float maxReach = L1 + L2;
  float minReach = fabs(L1 - L2); // Minimum reachable distance

  // Check if target is unreachable
  if (dist > maxReach || dist < minReach) {
    Serial.println("Target is unreachable.");
    baseDeg = getBaseAngleDeg(); // Keep current base angle
    elbowDeg = getElbowAngleDeg(); // Keep current elbow angle
    return;
  }

  // Calculate elbow angle (two configurations: elbow up and elbow down)
  float cosElbow = (dist * dist - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  if (cosElbow > 1.0f) cosElbow = 1.0f;
  if (cosElbow < -1.0f) cosElbow = -1.0f;
  float elbowRadUp = acosf(cosElbow); // Elbow up, positive
  float elbowRadDown = -acosf(cosElbow); // Elbow down, negative

  // Calculate base angle for both configurations
  float baseRadUp = atan2f(localY, localX) - atan2f(L2 * sinf(elbowRadUp), L1 + L2 * cosf(elbowRadUp));
  float baseRadDown = atan2f(localY, localX) - atan2f(L2 * sinf(elbowRadDown), L1 + L2 * cosf(elbowRadDown));

  // Convert to degrees
  float baseDegUp = baseRadUp * 180.0f / PI;
  float baseDegDown = baseRadDown * 180.0f / PI;
  float elbowDegUp = elbowRadUp * 180.0f / PI;
  float elbowDegDown = elbowRadDown * 180.0f / PI;

  // Decide which configuration to use
  bool useElbowUp = true; // Default to elbow up

  // Rule 1: Avoid fully extended positions
  if (fabs(elbowDegUp - 180.0f) < 10.0f) { // Threshold of 10° from 180°
    useElbowUp = false; // Switch to elbow down
  }

  // Rule 2: Prefer elbow down for targets below the base
  if (localY < 0) { // Target is below the base
    useElbowUp = false; // Switch to elbow down
  }

  // Rule 3: Prefer elbow up for targets above the base
  if (localY > 0) { // Target is above the base
    useElbowUp = true; // Switch to elbow up
  }

  // Apply the chosen configuration
  if (useElbowUp) {
    baseDeg = baseDegUp;
    elbowDeg = elbowDegUp;
  } else {
    baseDeg = baseDegDown;
    elbowDeg = elbowDegDown;
  }
  // Clamp elbow angle to avoid fully extended position
  if (elbowDeg > 160.0f) elbowDeg = 160.0f;

  Serial.print("IK Debug: localX="); Serial.print(localX); Serial.print(", localY="); Serial.print(localY);
  Serial.print(", dist="); Serial.print(dist); Serial.print(", elbowRadUp="); Serial.print(elbowRadUp);
  Serial.print(", elbowRadDown="); Serial.print(elbowRadDown); Serial.print(", baseRadUp="); Serial.print(baseRadUp);
  Serial.print(", baseRadDown="); Serial.println(baseRadDown);
}

/*
 * pidJointControl - PID-based control loop for base and elbow joints.
 * This function continuously adjusts the joint angles towards the provided target angles (thetaBaseRef and thetaElbowRef).
 * It gradually steps the current targets, computes the PID error (with integral anti-windup and derivative terms),
 * adds a feedforward torque, and limits the output to valid motor ranges. The loop runs until both joints are within
 * a defined tolerance of their targets, at which point the motors are stopped.
 */

static void pidJointControl(float thetaBaseRef, float thetaElbowRef) {
  float baseIntegral = 0.0;
  float elbowIntegral = 0.0;
  float lastBaseError = 0.0;
  float lastElbowError = 0.0;
  unsigned long lastTime = millis();
  float currentBaseTarget = getBaseAngleDeg();
  float currentElbowTarget = getElbowAngleDeg();

  while (true) {
    unsigned long now = millis();
    double dt = (now - lastTime) / 1000.0;
    if (dt <= 0.0) dt = 0.01;

    float baseAngle = getBaseAngleDeg();
    float elbowAngle = getElbowAngleDeg();

    // Step towards target angles
    if (fabs(thetaBaseRef - currentBaseTarget) > angleTolerance) {
      float baseStep = thetaBaseRef - currentBaseTarget;
      if (baseStep > MAX_ANGLE_STEP) baseStep = MAX_ANGLE_STEP;
      else if (baseStep < -MAX_ANGLE_STEP) baseStep = -MAX_ANGLE_STEP;
      currentBaseTarget += baseStep;
    } else {
      currentBaseTarget = thetaBaseRef;
    }

    if (fabs(thetaElbowRef - currentElbowTarget) > angleTolerance) {
      float elbowStep = thetaElbowRef - currentElbowTarget;
      if (elbowStep > MAX_ANGLE_STEP) elbowStep = MAX_ANGLE_STEP;
      else if (elbowStep < -MAX_ANGLE_STEP) elbowStep = -MAX_ANGLE_STEP;
      currentElbowTarget += elbowStep;
    } else {
      currentElbowTarget = thetaElbowRef;
    }

    // Calculate errors
    float baseError = currentBaseTarget - baseAngle;
    float elbowError = currentElbowTarget - elbowAngle;

    // Deadband: Ignore very small errors
    if (fabs(baseError) < 1.0) baseError = 0;
    if (fabs(elbowError) < 1.0) elbowError = 0;

    // Check if target is reached (error within tolerance)
    if (fabs(baseError) < angleTolerance && fabs(elbowError) < angleTolerance) {
      motorBase.run(0);
      motorElbow.run(0);
      Serial.println(baseAngle);
      Serial.println(elbowAngle);
      Serial.println("Target reached.");
      break;
    }

    // PID calculations
    baseIntegral += baseError * dt;
    elbowIntegral += elbowError * dt;

    // Anti-windup for integral term
    if (baseIntegral > 50.0) baseIntegral = 50.0;
    if (baseIntegral < -50.0) baseIntegral = -50.0;
    if (elbowIntegral > 50.0) elbowIntegral = 50.0;
    if (elbowIntegral < -50.0) elbowIntegral = -50.0;

    float baseDeriv = (baseError - lastBaseError) / dt;
    float elbowDeriv = (elbowError - lastElbowError) / dt;

    double baseOutput = baseKp * baseError + baseKi * baseIntegral + baseKd * baseDeriv;
    double elbowOutput = elbowKp * elbowError + elbowKi * elbowIntegral + elbowKd * elbowDeriv;

    // Feedforward term to provide additional torque
    float baseFeedforward = 60.0; // Adjust this value based on your system
    float elbowFeedforward = 60.0; // Adjust this value based on your system

    baseOutput += baseFeedforward;
    elbowOutput += elbowFeedforward;


    if (baseOutput > 255) baseOutput = 255;
    if (baseOutput < -255) baseOutput = -255;
    if (elbowOutput > 255) elbowOutput = 255;
    if (elbowOutput < -255) elbowOutput = -255;

    // Debug output
    Serial.print("baseAngle="); Serial.print(baseAngle); Serial.print(", elbowAngle="); Serial.print(elbowAngle);
    Serial.print(", baseError="); Serial.print(baseError); Serial.print(", elbowError="); Serial.print(elbowError);
    Serial.print(", baseOutput="); Serial.print((int)baseOutput); Serial.print(", elbowOutput="); Serial.println((int)elbowOutput);

    // Apply outputs to motors
    motorBase.run((int)baseOutput);
    motorElbow.run((int)elbowOutput);

    // Update last errors and time
    lastBaseError = baseError;
    lastElbowError = elbowError;
    lastTime = now;
    delay(50); // Small delay to prevent overloading the loop
  }
}

static float getPenAngleDeg() {
  float revs = (float)penEncoderCount / PEN_COUNTS_PER_REV ;
  return revs * 360.0f + 55.0f;
}

static float getBaseAngleDeg() {
  float revs = (float)baseEncoderCount / BASE_COUNTS_PER_REV;
  return revs * 360.0f;
}

static float getElbowAngleDeg() {
  float revs = (float)elbowEncoderCount / ELBOW_COUNTS_PER_REV;
  return revs * 360.0f;
}

static void baseEncISR() {
  int aVal = digitalRead(BASE_ENC_A);
  int bVal = digitalRead(BASE_ENC_B);
  if (aVal == bVal) baseEncoderCount--; // CW decreases
  else baseEncoderCount++;              // CCW increases
}

static void elbowEncISR() {
  int aVal = digitalRead(ELBOW_ENC_A);
  int bVal = digitalRead(ELBOW_ENC_B);
  if (aVal == bVal) elbowEncoderCount--; // CW decreases
  else elbowEncoderCount++;              // CCW increases
}

static void penEncISR() {
  int aVal = digitalRead(PEN_ENC_A);
  int bVal = digitalRead(PEN_ENC_B);
  if (aVal == bVal) penEncoderCount--; // CW decreases
  else penEncoderCount++;              // CCW increases
}
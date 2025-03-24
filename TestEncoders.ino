/***************************************************
 *                TestEncoders.ino
 *
 * This code tests the encoder readings for the
 * 2D arm system. It prints out the raw encoder 
 * counts and computed angles so you can manually 
 * move the arms and note the limits.

Autors:
Cassidy Berzuk
Titus Waldner
 ***************************************************/

#include <Arduino.h>
#include <math.h>

// -------------------- Encoder Pin Definitions --------------------
// For the base (Port 2)
const byte BASE_ENC_A = 18;    // Base joint encoder channel A (external interrupt pin)
const byte BASE_ENC_B = 31;    // Base joint encoder channel B

// For the arm (Port 3)
const byte ELBOW_ENC_A = 3;    // Arm joint encoder channel A (external interrupt pin)
const byte ELBOW_ENC_B = 49;   // Arm joint encoder channel B

// -------------------- Global Variables --------------------
volatile long baseEncoderCount = 0;   // Accumulated encoder counts for the base joint
volatile long elbowEncoderCount = 0;  // Accumulated encoder counts for the elbow joint

// -------------------- Counts-Per-Revolution--------------------
const float BASE_COUNTS_PER_REV = 1398.0;
const float ELBOW_COUNTS_PER_REV = 833.0;

// -------------------- Encoder Interrupt Service Routines --------------------
void baseEncISR() {
  // Simple quadrature decoding for base joint
  int aVal = digitalRead(BASE_ENC_A);
  int bVal = digitalRead(BASE_ENC_B);
  if (aVal == bVal) {
    baseEncoderCount++;
  } else {
    baseEncoderCount--;
  }
}

void elbowEncISR() {
  // Simple quadrature decoding for elbow joint
  int aVal = digitalRead(ELBOW_ENC_A);
  int bVal = digitalRead(ELBOW_ENC_B);
  if (aVal == bVal) {
    elbowEncoderCount++;
  } else {
    elbowEncoderCount--;
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(1000); // Allow time for serial monitor to open

  // Set encoder pins as inputs with pull-ups
  pinMode(BASE_ENC_A, INPUT_PULLUP);
  pinMode(BASE_ENC_B, INPUT_PULLUP);
  pinMode(ELBOW_ENC_A, INPUT_PULLUP);
  pinMode(ELBOW_ENC_B, INPUT_PULLUP);

  // Attach interrupts for the primary encoder channels.
  // On the Arduino Mega, pins 18 and 3 support external interrupts.
  attachInterrupt(digitalPinToInterrupt(BASE_ENC_A), baseEncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELBOW_ENC_A), elbowEncISR, CHANGE);

  Serial.println("Encoder Test Initialized.");
  Serial.println("Manually move the arms and observe the values below:");
  Serial.println();
}

// -------------------- Loop --------------------
void loop() {
  // Compute angles in degrees: (counts / counts-per-rev) * 360
  float baseAngle = (baseEncoderCount / BASE_COUNTS_PER_REV) * 360.0;
  float elbowAngle = (elbowEncoderCount / ELBOW_COUNTS_PER_REV) * 360.0;

  // Print raw counts and computed angles
  Serial.print("Base Count: ");
  Serial.print(baseEncoderCount);
  Serial.print(" | Base Angle: ");
  Serial.print(baseAngle, 2);
  Serial.print(" deg\t");

  Serial.print("Elbow Count: ");
  Serial.print(elbowEncoderCount);
  Serial.print(" | Elbow Angle: ");
  Serial.print(elbowAngle, 2);
  Serial.println(" deg");

  delay(500);  // Update every 500 ms
}

/*
 * Autonomous Crawler Control System
 * Features: Photo-Interrupter distance tracking & Gyro-stabilized heading.
 * Sensors: SK-Gyro (Voltage Oscillation), Photo-Interrupter (Pin 11).
 */

// --- Pin Definitions ---
const int photoInterrupterPin = 11; // Sensor for wheel revolution counting
const int gyroPin = A0;             // Analog input from SK-Gyro

// Motor Driver Pins (Example: L298N)
const int motorA1 = 5; 
const int motorA2 = 6;
const int motorB1 = 9;
const int motorB2 = 10;

// --- Global Variables ---
volatile int pulseCount = 0;       // Counts revolutions from photo-interrupter
float currentHeading = 0.0;        // Calculated angle from Gyro
unsigned long lastTime;

void setup() {
  Serial.begin(9600);
  
  // Initialize Sensor Pins
  pinMode(photoInterrupterPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(photoInterrupterPin), countPulse, FALLING);
  
  // Initialize Motor Pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  lastTime = millis();
  Serial.println("System Initialized: Crawler Ready.");
}

void loop() {
  // Example Sequence: Move forward for 50 pulses, then rotate 90 degrees.
  moveForward(50, 150); // 50 pulses at speed 150
  stopMotors();
  delay(1000);
  
  rotateRight(90);      // Rotate 90 degrees using Gyro
  stopMotors();
  delay(1000);
}

// --- Movement Functions ---

/**
 * Moves the crawler forward while using the Gyro to stay straight.
 * @param targetPulses: Distance to travel based on wheel revolutions.
 * @param baseSpeed: PWM speed (0-255).
 */
void moveForward(int targetPulses, int baseSpeed) {
  pulseCount = 0;
  float initialHeading = getGyroHeading();

  while (pulseCount < targetPulses) {
    float error = getGyroHeading() - initialHeading;
    int correction = error * 5; // Simple P-Correction for steering

    analogWrite(motorA1, baseSpeed - correction);
    analogWrite(motorB1, baseSpeed + correction);
    
    // Log output for debugging
    Serial.print("Distance: "); Serial.print(pulseCount);
    Serial.print(" | Gyro Error: "); Serial.println(error);
  }
}

void moveBackward(int targetPulses, int baseSpeed) {
  pulseCount = 0;
  while (pulseCount < targetPulses) {
    digitalWrite(motorA1, LOW);
    analogWrite(motorA2, baseSpeed);
    digitalWrite(motorB1, LOW);
    analogWrite(motorB2, baseSpeed);
  }
}

/**
 * Rotates the crawler to a specific angle using Gyro feedback.
 */
void rotateRight(float targetAngle) {
  float startAngle = getGyroHeading();
  while (abs(getGyroHeading() - startAngle) < targetAngle) {
    analogWrite(motorA1, 150);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    analogWrite(motorB2, 150);
  }
}

// --- Sensor Processing ---

void countPulse() {
  pulseCount++;
}

float getGyroHeading() {
  // Reads the voltage from the SK-Gyro and converts to angular velocity
  int rawValue = analogRead(gyroPin);
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  
  // Simple integration of angular velocity to get heading
  // (Adjust 'sensitivity' based on your SK-Gyro datasheet)
  float sensitivity = 0.67; 
  float voltage = (rawValue * 5.0) / 1023.0;
  float rate = (voltage - 2.5) / sensitivity; 
  
  currentHeading += rate * deltaTime;
  lastTime = currentTime;
  
  return currentHeading;
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}
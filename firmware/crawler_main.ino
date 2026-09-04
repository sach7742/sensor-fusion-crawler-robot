/**
 * @file crawler_main.ino
 * @brief Autonomous Crawler Robot with Sensor-Fusion State Machine
 * @details Implements non-blocking control loops for MPU-6050 yaw tracking
 *          and photoelectric obstacle avoidance using L298N H-Bridge driver.
 */

#include <Wire.h>

// ============================================================================
// Hardware Pinout Configuration (Matches docs/pinout.md)
// ============================================================================
namespace Config {
    // I2C Address for MPU-6050
    constexpr uint8_t MPU6050_ADDR = 0x68;

    // Photoelectric / Infrared Digital Sensor Pins
    constexpr uint8_t PIN_SENSOR_LEFT  = 2;
    constexpr uint8_t PIN_SENSOR_RIGHT = 3;

    // L298N Motor Driver Control Pins
    constexpr uint8_t PIN_ENA  = 5;   // Left Track Speed (PWM)
    constexpr uint8_t PIN_ENB  = 6;   // Right Track Speed (PWM)
    constexpr uint8_t PIN_IN1  = 7;   // Left Track Dir 1
    constexpr uint8_t PIN_IN2  = 8;   // Left Track Dir 2
    constexpr uint8_t PIN_IN3  = 9;   // Right Track Dir 1
    constexpr uint8_t PIN_IN4  = 10;  // Right Track Dir 2

    // Motion & Control Constants
    constexpr uint8_t BASE_SPEED       = 180; // Default PWM duty cycle (0-255)
    constexpr uint8_t TURN_SPEED       = 150;
    constexpr float YAW_THRESHOLD_DEG  = 5.0f; // Max allowable heading drift
    constexpr unsigned long CONTROL_LOOP_INTERVAL_MS = 20; // 50Hz control loop
}

// ============================================================================
// State Machine & Telemetry Types
// ============================================================================
enum class RobotState {
    FORWARD,
    CORRECT_LEFT,
    CORRECT_RIGHT,
    AVOID_OBSTACLE,
    STOP
};

struct SensorData {
    bool obstacleLeft;
    bool obstacleRight;
    float currentYaw;
    float gyroZ;
};

// Global Execution Variables
RobotState currentState = RobotState::FORWARD;
SensorData sensors = {false, false, 0.0f, 0.0f};
unsigned long lastControlTime = 0;
int16_t rawGyroZ = 0;
float yawAngle = 0.0f;

// ============================================================================
// Low-Level Hardware Interface Subroutines
// ============================================================================

void initMPU6050() {
    Wire.begin();
    Wire.beginTransmission(Config::MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // Set to zero (wakes up MPU-6050)
    Wire.endTransmission(true);
}

void readGyroData() {
    Wire.beginTransmission(Config::MPU6050_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(Config::MPU6050_ADDR, (uint8_t)2, (uint8_t)true);

    if (Wire.available() >= 2) {
        rawGyroZ = (Wire.read() << 8) | Wire.read();
        // Convert raw sensitivity factor (FS_SEL=0 -> 131 LSB/(deg/s))
        sensors.gyroZ = static_cast<float>(rawGyroZ) / 131.0f;
    }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Left Track Direction & Speed
    if (leftSpeed >= 0) {
        digitalWrite(Config::PIN_IN1, HIGH);
        digitalWrite(Config::PIN_IN2, LOW);
    } else {
        digitalWrite(Config::PIN_IN1, LOW);
        digitalWrite(Config::PIN_IN2, HIGH);
        leftSpeed = -leftSpeed;
    }

    // Right Track Direction & Speed
    if (rightSpeed >= 0) {
        digitalWrite(Config::PIN_IN3, HIGH);
        digitalWrite(Config::PIN_IN4, LOW);
    } else {
        digitalWrite(Config::PIN_IN3, LOW);
        digitalWrite(Config::PIN_IN4, HIGH);
        rightSpeed = -rightSpeed;
    }

    analogWrite(Config::PIN_ENA, constrain(leftSpeed, 0, 255));
    analogWrite(Config::PIN_ENB, constrain(rightSpeed, 0, 255));
}

// ============================================================================
// Core Sensor Fusion & State Machine Logic
// ============================================================================

void updateSensors(float deltaTimeSec) {
    // Digital Low active reading for proximity modules
    sensors.obstacleLeft  = (digitalRead(Config::PIN_SENSOR_LEFT) == LOW);
    sensors.obstacleRight = (digitalRead(Config::PIN_SENSOR_RIGHT) == LOW);

    // Integrate Gyro Z rate to track heading yaw
    readGyroData();
    yawAngle += sensors.gyroZ * deltaTimeSec;
    sensors.currentYaw = yawAngle;
}

void evaluateStateMachine() {
    // Obstacle Detection takes priority over yaw trajectory alignment
    if (sensors.obstacleLeft || sensors.obstacleRight) {
        currentState = RobotState::AVOID_OBSTACLE;
        return;
    }

    // Evaluate Heading Trajectory Correction via Gyroscope Integration
    if (sensors.currentYaw > Config::YAW_THRESHOLD_DEG) {
        currentState = RobotState::CORRECT_RIGHT;
    } else if (sensors.currentYaw < -Config::YAW_THRESHOLD_DEG) {
        currentState = RobotState::CORRECT_LEFT;
    } else {
        currentState = RobotState::FORWARD;
    }
}

void executeMotionCommand() {
    switch (currentState) {
        case RobotState::FORWARD:
            setMotorSpeeds(Config::BASE_SPEED, Config::BASE_SPEED);
            break;

        case RobotState::CORRECT_LEFT:
            // Compensate drift by increasing left track drive
            setMotorSpeeds(Config::BASE_SPEED + 30, Config::BASE_SPEED - 30);
            break;

        case RobotState::CORRECT_RIGHT:
            // Compensate drift by increasing right track drive
            setMotorSpeeds(Config::BASE_SPEED - 30, Config::BASE_SPEED + 30);
            break;

        case RobotState::AVOID_OBSTACLE:
            // Reverse away from immediate boundary hazard
            if (sensors.obstacleLeft) {
                setMotorSpeeds(-Config::TURN_SPEED, Config::TURN_SPEED); // Spin right
            } else {
                setMotorSpeeds(Config::TURN_SPEED, -Config::TURN_SPEED); // Spin left
            }
            break;

        case RobotState::STOP:
        default:
            setMotorSpeeds(0, 0);
            break;
    }
}

// ============================================================================
// Main Arduino Setup & Non-Blocking Execution Loop
// ============================================================================

void setup() {
    // Configure Motor Driver GPIO Output Pins
    pinMode(Config::PIN_ENA, OUTPUT);
    pinMode(Config::PIN_ENB, OUTPUT);
    pinMode(Config::PIN_IN1, OUTPUT);
    pinMode(Config::PIN_IN2, OUTPUT);
    pinMode(Config::PIN_IN3, OUTPUT);
    pinMode(Config::PIN_IN4, OUTPUT);

    // Configure Sensor Digital Inputs
    pinMode(Config::PIN_SENSOR_LEFT, INPUT);
    pinMode(Config::PIN_SENSOR_RIGHT, INPUT);

    initMPU6050();
    lastControlTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastControlTime;

    // Non-blocking deterministic tick execution (50 Hz interval)
    if (elapsedTime >= Config::CONTROL_LOOP_INTERVAL_MS) {
        float deltaTimeSec = static_cast<float>(elapsedTime) / 1000.0f;
        lastControlTime = currentTime;

        updateSensors(deltaTimeSec);
        evaluateStateMachine();
        executeMotionCommand();
    }
}

# Sensor-Fusion Autonomous Crawler Robot

An embedded mechatronics system featuring sensor-guided autonomous navigation, dynamic obstacle avoidance, and dynamic trajectory correction using C++ on microcontrollers.

---

## Technical Overview

* **Closed-Loop Motion Control**: Real-time yaw correction and angular velocity tracking via MPU-6050 6-DOF IMU integration.
* **Proximity Polling**: Dual-photoelectric/infrared sensors for edge detection and collision avoidance.
* **Modular Firmware Architecture**: Clean separation of sensor driver subroutines, motor control, and state machine loops.

---

## Hardware Architecture & Bill of Materials

| Component | Specification / Model | Function | Interface / Protocol |
| --- | --- | --- | --- |
| Microcontroller | ATmega328P / Arduino Uno | Core execution loop & sensor polling | GPIO / PWM / I2C |
| Orientation Sensor | MPU-6050 (Gyro + Accelerometer) | Yaw detection & trajectory tracking | I2C (`0x68`) |
| Edge / Proximity Sensors | Dual Photoelectric / IR Modules | Boundary detection & obstacle avoidance | Digital GPIO Interrupts |
| Motor Driver | L298N Dual H-Bridge | PWM track drive motor control | Digital / PWM |
| Chassis | Custom Track-Drive System | Differential steering & multi-terrain drive | Mechanical |

---

## Firmware Directory Structure

```text
sensor-fusion-crawler-robot/
├── docs/                 # Hardware schematics, wiring pinouts, and system specs
│   └── pinout.md         # Microcontroller-to-sensor pin mapping
├── firmware/             # Embedded C++ / Arduino source files
│   └── crawler_main.ino  # Main control loop & state machine logic
├── LICENSE               # Open-source license (MIT)
└── README.md             # Technical documentation & project specification

```

---

## System Block Diagram

```text
  ┌─────────────────────────────┐        ┌──────────────────────────────┐
  │  MPU-6050 Gyro / Accelerometer  │        │ Dual Photoelectric / IR Sensors │
  └──────────────┬──────────────┘        └──────────────┬───────────────┘
                 │ I2C Signals                          │ Digital Logic Pins
                 ▼                                      ▼
  ┌─────────────────────────────────────────────────────────────────────┐
  │                     ATmega328P Microcontroller                       │
  │                  (C++ State Machine & Controller)                   │
  └──────────────────────────────────┬──────────────────────────────────┘
                                     │ PWM Duty Cycle / Direction Logic
                                     ▼
  ┌─────────────────────────────────────────────────────────────────────┐
  │                        L298N Motor Driver                           │
  └──────────────────────────────────┬──────────────────────────────────┘
                                     │ High-Current Drive
                                     ▼
  ┌─────────────────────────────────────────────────────────────────────┐
  │                   Dual-Track Crawler Mechanism                      │
  └──────────────────────────────────┬──────────────────────────────────┘

```

---

## Getting Started

### Prerequisites

* Arduino IDE 2.0+ or PlatformIO Core
* AVR Board Support Packages

### Build & Flash

1. Clone the repository:
```bash
git clone [https://github.com/sach7742/sensor-fusion-crawler-robot.git](https://github.com/sach7742/sensor-fusion-crawler-robot.git)
cd sensor-fusion-crawler-robot

```


2. Open `firmware/crawler_main.ino` in your IDE.
3. Target your board (e.g., Arduino Uno) and compile/upload.

```


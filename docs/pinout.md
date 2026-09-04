# Hardware Pinout & Wiring Specification

## Microcontroller Pin Mapping

| Microcontroller Pin | Connected Hardware Module | Component Pin | Signal Type | Description |
| --- | --- | --- | --- | --- |
| **A4** | MPU-6050 Gyro/Accel | SDA | I2C Data | Serial Data line for IMU telemetry |
| **A5** | MPU-6050 Gyro/Accel | SCL | I2C Clock | Serial Clock line for IMU telemetry |
| **D2** | Left Photoelectric Sensor | OUT | Digital Input | Proximity detection (Interrupt capable) |
| **D3** | Right Photoelectric Sensor | OUT | Digital Input | Proximity detection (Interrupt capable) |
| **D5** | L298N Motor Driver | ENA | PWM Output | Left track speed control (PWM) |
| **D6** | L298N Motor Driver | ENB | PWM Output | Right track speed control (PWM) |
| **D7** | L298N Motor Driver | IN1 | Digital Output | Left track direction control |
| **D8** | L298N Motor Driver | IN2 | Digital Output | Left track direction control |
| **D9** | L298N Motor Driver | IN3 | Digital Output | Right track direction control |
| **D10** | L298N Motor Driver | IN4 | Digital Output | Right track direction control |

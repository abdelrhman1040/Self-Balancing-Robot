# Firmware Documentation

## Overview
The firmware is built on the Arduino Framework specifically for the ESP32 Dual-Core architecture. It is designed to decouple critical real-time control loops from non-deterministic Wi-Fi and Web Server tasks. This separation ensures that heavy network traffic or web dashboard updates do not interrupt the PID loop, maintaining consistent stability for the robot.

---

## System Architecture

The firmware utilizes FreeRTOS to distribute tasks across the two cores of the ESP32:

### 1. Core 1: Real-Time Control Task (High Priority)
This core is dedicated to the physics and stability of the robot.
* Frequency: ~100Hz (10ms loop time).
* Responsibilities:
    * Sensor Fusion: Reads absolute orientation data from the BNO055 IMU via I2C.
    * PID Calculation: Computes the Cascaded PID output (Balance Inner Loop + Speed Outer Loop).
    * Motor Drive: Generates step pulses using the AccelStepper library with acceleration limiting.
    * Safety Checks: Monitors tilt angle (automatic cutoff at >45 degrees) and sensor connectivity.

### 2. Core 0: Communication Task (Low Priority)
This core handles user interaction and network management.
* Responsibilities:
    * Wi-Fi Management: Handles connection stability and re-connection logic.
    * Web Server: Serves the HTML/JavaScript Dashboard to the browser.
    * API Handling: Processes AJAX requests for real-time tuning and telemetry data.
    * NVS Storage: Manages reading/writing tuning profiles to Flash memory via the Preferences library.

---

## Dependencies and Libraries

The following libraries are required to compile the project:

| Library | Purpose |
| :--- | :--- |
| Adafruit BNO055 | Driver for the IMU sensor to read Euler angles. |
| Adafruit Unified Sensor | Base dependency for Adafruit sensor libraries. |
| AccelStepper | Multi-stepper control with acceleration support. |
| Preferences | Non-Volatile Storage (NVS) management on ESP32. |
| WebServer | Standard library for handling HTTP requests. |

---

## Pin Definitions and Hardware Compatibility

The firmware defaults to the configuration for PCB Version 1 (V1).

### Current Pinout (V1 Default)
```cpp
#define STEP_PIN_1   14   // Left Motor Step
#define DIR_PIN_1    12   // Left Motor Dir
#define STEP_PIN_2   26   // Right Motor Step
#define DIR_PIN_2    27   // Right Motor Dir
#define SDA_PIN      21   // BNO055 SDA
#define SCL_PIN      22   // BNO055 SCL

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
```

---

## Important for PCB V2 Users
The PCB Version 2 design avoids using GPIO 2 to prevent boot-strapping issues. If you are deploying this firmware on a V2 board, you must update the pin definitions in the source code to match the V2 schematic before uploading. Failure to do so may result in boot loops or unresponsive motors.

---

## Troubleshooting and System Stability

### Brownout Detector Issue (PCB V1)
In the initial PCB design (V1), the power path is shared between high-current stepper drivers and the ESP32 logic without complete electrical isolation.

**The Problem:**
Rapid motor acceleration causes significant current spikes, leading to a momentary voltage drop on the 3.3V rail. The ESP32's internal Brownout Detector interprets this as a power failure and resets the chip, causing a continuous boot loop during operation.

**The Software Solution:**
If your robot resets whenever it tries to balance or move (especially on PCB V1), the code includes a fix to disable the Brownout Detector at the start of the setup function:

```cpp
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

void setup() {
    // Disabling Brownout Detector to allow operation during current spikes
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    
    // ... remaining setup logic
}
```

> **Note:** While this software fix allows the robot to run, ensure your 3S LiPo battery is adequately charged to maintain voltage stability.

---

## API Endpoints (Web Interface)
The Web Dashboard interacts with the ESP32 through HTTP GET requests.

| Endpoint | Description |
| :--- | :--- |
| **`GET /data`** | Returns a JSON object with live telemetry (Angle, Speed, PID components). |
| **`GET /set?k=[KEY]&v=[VALUE]`** | Updates PID constants instantly.<br>**Keys:** `bp` (Balance P), `bi` (Balance I), `bd` (Balance D), `sp` (Setpoint). |
| **`GET /save?p=[SLOT]`** | Saves current tuning to Slot 1, 2, or 3 in Flash memory. |
| **`GET /move?d=[DIRECTION]`** | Sends movement commands.<br>**Values:** `F` (Forward), `B` (Backward), `L` (Left), `R` (Right). |

![WhatsApp Image 2026-02-13 at 10 07 07 PM](https://github.com/user-attachments/assets/84be06c6-2128-47be-b120-cf115ded749b)



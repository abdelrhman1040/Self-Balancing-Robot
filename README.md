# ESP32 Self-Balancing Robot 🤖
### A Web-Controlled, Real-Time Tunable Inverted Pendulum Platform

![Project Banner](https://via.placeholder.com/800x400?text=Place+Your+Robot+Image+Here)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/Platform-ESP32-blue)](https://espressif.com)
[![Framework](https://img.shields.io/badge/Framework-Arduino%2FPlatformIO-green)](https://platformio.org)

---

## 📖 Project Overview

This project implements a self-balancing robot based on the classical inverted pendulum problem. The system is inherently unstable and requires continuous feedback control to maintain its upright position.

Unlike standard implementations, this robot features a **Web-based Dashboard** hosted on the ESP32, allowing for **real-time PID tuning** and telemetry visualization over Wi-Fi without needing to re-upload code.

### Key Objectives
* **Mathematical Modeling:** Linearization and transfer function derivation.
* **Control System Design:** Implementation of cascaded PID loops.
* **Embedded Firmware:** High-frequency control loop on ESP32 (Dual Core).
* **Web Interface:** Real-time tuning and plotting using WebSockets/AJAX.
* **Hardware:** Custom PCB design and 3D printed mechanical chassis.

### Applications
* Autonomous robotics platforms.
* Mobile balancing transport systems.
* Educational control systems laboratories.

---

## 📂 Repository Structure

While this README provides the complete documentation, detailed assets are organized as follows:

* **[`/Firmware`](./Firmware)**: Source code (PlatformIO/Arduino) and libraries.
* **[`/PCB`](./PCB)**: EasyEDA Gerber files, schematics, and BOM.
* **[`/Mechanical`](./Mechanical)**: STL files for 3D printing and CAD models.
* **[`/Simulation`](./Simulation)**: MATLAB/Simulink scripts for system validation.

---

## 🛠️ Building the Robot (Hardware)

This section describes how to physically reproduce the robot.

### 1. Mechanical Assembly
1.  **Chassis:** 3D print the chassis using the STL files in the `/Mechanical` folder.
2.  **Motors:** Mount the NEMA17 stepper motors in their designated slots.
3.  **Drive Train:** Attach the wheels securely to the motor shafts.
4.  **Electronics Mount:** Fix the PCB in its mounting slots using M3 screws.
5.  **IMU Placement:** Place the **BNO055 IMU** at the exact physical center of the robot to minimize centrifugal acceleration errors.
6.  **Battery:** Install the battery holder (ensure it is secure).

> **⚠️ Critical Note:** Ensure the **Center of Mass (CoM)** is aligned vertically above the wheel axis. Poor mass distribution acts as a constant disturbance and degrades controller performance.

### 2. Electrical Assembly
Connect the components according to the schematic in `/PCB`:

* **MCU:** ESP32 Development Board.
* **Actuators:** 2x Stepper Motors via A4988/DRV8825 drivers.
* **Sensor:** BNO055 IMU via I2C (SDA, SCL).
* **Power:** Buck converter (12V $\to$ 5V) for logic; direct battery power for motors.

**Pre-Flight Checks:**
* [ ] Check Motor Driver current limiting (Vref adjustment).
* [ ] Verify common ground between logic and power circuits.
* [ ] Secure I2C wiring (short wires are better to avoid noise).

---

## 🚀 Getting Started (Firmware & Wi-Fi)

### Firmware Upload
1.  Open the project in **PlatformIO** (Recommended) or Arduino IDE.
2.  Navigate to `src/main.cpp` or `config.h` (depending on your structure).
3.  **Update Wi-Fi Credentials:**
    ```cpp
    const char* ssid = "your_wifi_name";
    const char* password = "your_wifi_password";
    ```
4.  Upload the code to the ESP32.

### System Launch
1.  Open the **Serial Monitor** (Baud Rate: 115200).
2.  Reset the ESP32.
3.  Wait for the connection confirmation message:
    ```text
    Connected to Wi-Fi!
    IP Address: 192.168.1.15
    ```
4.  Open your web browser (Phone or PC) and enter the **IP Address**.
5.  The **Control Dashboard** will load, allowing you to start the robot and tune parameters.

---

## 🧮 Mathematical Modeling

The robot behaves as an inverted pendulum mounted on a wheeled base. The simplified linearized equation of motion around the upright equilibrium ($\theta \approx 0$) assuming negligible cart acceleration is:

$$
(I + mL^2)\ddot{\theta} - mgL\theta = d(\theta)
$$

Where:
* $I$: Moment of inertia.
* $m$: Mass of the pendulum.
* $L$: Distance to center of mass.
* $g$: Gravitational acceleration.

### Transfer Function
Taking the Laplace transform, the open-loop transfer function is:

$$
\frac{\theta(s)}{D(s)} = \frac{1}{s^2 - a} \quad \text{where} \quad a = \frac{mgL}{I + mL^2}
$$

This reveals a pole in the Right-Half Plane ($s = \pm \sqrt{a}$), mathematically proving the system is **unstable** and requires closed-loop control.

---

## 🕹️ Control Architecture

The ESP32 firmware implements two **Cascaded PID Loops** to ensure stability and position control.

### 1. Balance PID (Inner Loop)
This is the fast loop (running at ~200Hz+). It maintains the upright position.

$$
u_{bal}(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}
$$

```cpp
// Code Implementation
float term_bP = Kp * balanceError;
float term_bI = Ki * sumBalanceError;
float term_bD = Kd * (balanceError - lastBalanceError);

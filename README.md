# Self-Balancing Robot
### Inverted Pendulum Modeling, Embedded Control, and Real-Time PID Tuning Platform

## Project Overview

This project implements a self-balancing robot based on the classical inverted pendulum problem. The system is inherently unstable and requires continuous feedback control to maintain its upright position.

Distinct from standard implementations, this robot features a **Web-based Dashboard** hosted directly on the ESP32. This allows for **real-time PID tuning** and telemetry visualization over Wi-Fi, eliminating the need to re-upload firmware for parameter adjustments.

### Key Features
* **Mathematical Modeling:** Linearization and transfer function derivation of the system.
* **Control System Design:** Implementation of cascaded PID loops for stability and drift control.
* **Embedded Firmware:** High-frequency control loop execution on the ESP32 dual-core architecture.
* **Web Interface:** Real-time tuning and data plotting using WebSockets.
* **Hardware Implementation:** Custom PCB design and 3D printed mechanical chassis.

### Applications
* Autonomous robotics platforms.
* Mobile balancing transport systems.
* Educational control systems laboratories.
* Real-time embedded control experimentation.

---

## Repository Contents

This repository is organized into the following directories:

* **[`/Firmware`](./Firmware)**: Source code (PlatformIO/Arduino) and libraries.
* **[`/PCB`](./PCB)**: EasyEDA Gerber files, schematics, and BOM.
* **[`/Mechanical`](./Mechanical)**: STL files for 3D printing and CAD models.
* **[`/Simulation`](./Simulation)**: MATLAB/Simulink scripts for system validation.

---

## Hardware Assembly

### 1. Mechanical Assembly

1.  **Chassis Fabrication:** 3D print the chassis using the STL files provided in the `/Mechanical` directory.
2.  **Motor Mounting:** Mount the NEMA17 stepper motors in the designated chassis slots.
3.  **Drive Train:** Securely attach the wheels to the motor shafts.
4.  **Electronics Mounting:** Fix the PCB in its mounting slots using M3 screws.
5.  **IMU Alignment:** Place the BNO055 IMU sensor at the physical center of the robot.
    * *Note:* Misalignment introduces centrifugal acceleration artifacts into the tilt reading.
6.  **Center of Mass:** Ensure the center of mass is aligned vertically above the wheel axis.

### 2. Electrical Assembly

Connect the components according to the schematic provided in the `/PCB` directory:

* **Microcontroller:** ESP32 Development Board.
* **Actuators:** Two Stepper Motors driven by A4988 or DRV8825 drivers.
* **Sensor:** BNO055 IMU communicating via I2C (SDA, SCL).
* **Power Management:** Buck converter (12V to 5V) for logic supply; direct battery connection for motor drivers.

**Verification Steps:**
* Calibrate motor driver current limits (Vref).
* Verify common ground continuity between logic and power circuits.
* Ensure I2C connections are secure and shielded.

---

## Firmware Configuration and Upload

To configure and upload the firmware:

1.  Open the project in **PlatformIO** or **Arduino IDE**.
2.  Navigate to the main configuration file (e.g., `config.h` or `main.cpp`).
3.  Update the Wi-Fi credentials:

    ```cpp
    const char* ssid = "your_wifi_name";
    const char* password = "your_wifi_password";
    ```

4.  Upload the firmware to the ESP32.
5.  Open the Serial Monitor (Baud Rate: 115200).
6.  Upon successful connection, the ESP32 will output its assigned IP address.
7.  Enter this IP address in a web browser to access the control dashboard.

---

## Mathematical Modeling

The robot is modeled as an inverted pendulum mounted on a wheeled cart. The nonlinear equations of motion are defined as follows:

**1. Cart Translation:**
$$
(M + m)\ddot{x} + mL \cos\theta \ddot{\theta} - mL\dot{\theta}^2 \sin\theta = F(x)
$$

**2. Pendulum Rotation:**
$$
mL \cos\theta \ddot{x} + (I + mL^2)\ddot{\theta} - mgL\sin\theta = d(\theta)
$$

These equations contain nonlinear terms ($\sin\theta$, $\cos\theta$, $\dot{\theta}^2$). For control design, the system is linearized around the upright equilibrium point ($\theta \approx 0$), assuming small angles ($\sin\theta \approx \theta$, $\cos\theta \approx 1$) and negligible cart acceleration.

The simplified equation becomes:
$$
(I + mL^2)\ddot{\theta} - mgL\theta = d(\theta)
$$

**Transfer Function:**
Applying the Laplace transform yields the open-loop transfer function:

$$
\frac{\theta(s)}{D(s)} = \frac{1}{s^2 - a}
$$

Where:
$$
a = \frac{mgL}{I + mL^2}
$$

The poles of the system are located at $s = \pm \sqrt{a}$. The presence of a pole in the right-half plane ($+\sqrt{a}$) indicates that the open-loop system is inherently unstable.

---

## Control Strategy

To stabilize the system, a PID controller is implemented. The control law in the Laplace domain is:

$$
C(s) = K_p + K_d s + \frac{K_i}{s}
$$

* **Proportional ($K_p$):** Provides immediate response to tilt error.
* **Derivative ($K_d$):** Adds "virtual friction" (damping) to reduce overshoot and oscillation.
* **Integral ($K_i$):** Eliminates steady-state error caused by mechanical asymmetries.

### Embedded Control Architecture

The firmware utilizes a **Cascaded Control Architecture**:

1.  **Balance Loop (Inner Loop):**
    * Operates at high frequency (~200Hz).
    * Calculates motor speed based on tilt angle.
    * Includes anti-windup and deadband compensation.

    ```cpp
    float term_bP = Kp * balanceError;
    float term_bI = Ki * sumBalanceError;
    float term_bD = Kd * (balanceError - lastBalanceError);
    ```

2.  **Speed/Drift Loop (Outer Loop):**
    * Operates at a lower frequency.
    * Adjusts the target angle setpoint to prevent linear drift across the floor.

    ```cpp
    float term_sP = speedOutput * Kp_s;
    float term_sI = sumSpeedError * Ki_s;
    float driftCorrection = term_sP + term_sI;
    ```

---

## Web Dashboard Interface

The ESP32 hosts an asynchronous web server providing a control interface with the following functionalities:

* **Parameter Tuning:** Sliders for adjusting $K_p$, $K_i$, and $K_d$ values for both balance and speed loops instantly.
* **Telemetry:** Live plotting of tilt angle, motor speed, and PID output using Chart.js.
* **Control:** Remote start/stop functionality.
* **Data Persistence:** Capability to save and load tuning profiles from non-volatile memory (EEPROM).

---

## Conclusion

This project demonstrates the complete engineering lifecycle of a control system, from mathematical modeling and linearization to hardware implementation and real-time embedded control. It serves as a robust platform for testing control algorithms and studying unstable system dynamics.



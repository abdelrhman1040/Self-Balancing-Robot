# Self-Balancing Robot

Inverted Pendulum Modeling, Embedded Control, and Real-Time PID Tuning Platform

---

## Project Overview

This project implements a self-balancing robot based on the classical inverted pendulum problem. The system is inherently unstable and requires continuous feedback control to maintain its upright position.

The goal of the project is not only to stabilize the robot, but to develop a complete engineering pipeline including:

* Mathematical modeling of the system
* Linearization and transfer function derivation
* Control system design
* Embedded firmware implementation on ESP32
* Real-time PID tuning through a web-based interface
* Mechanical design (3D CAD)
* Custom PCB design

Applications of this type of system include:

* Autonomous robotics platforms
* Mobile balancing transport systems
* Educational control systems laboratories
* Real-time embedded control experimentation

This repository contains the complete documentation, firmware, 3D models, and PCB design required to reproduce the system.

---

# Building the Robot from Scratch

This section describes how to physically reproduce the robot and bring it to life.

---

## 1. Mechanical Assembly

1. 3D print the chassis using the provided CAD files.
2. Mount the stepper motors in their designated positions.
3. Attach the wheels securely to the motor shafts.
4. Fix the PCB in its mounting slots.
5. Install the battery holder.
6. Place the IMU sensor (BNO055) at the correct central location to minimize vibration effects.
7. Ensure the center of mass is aligned vertically above the wheel axis.

Correct mechanical alignment is critical. Poor mass distribution directly affects controller performance.

---

## 2. Electrical Assembly

Connect the following components:

* ESP32 microcontroller
* Two stepper motors via A4988 drivers
* BNO055 IMU via I2C
* Buck converter for voltage regulation
* Battery pack

Check:

* Motor driver current limiting
* Proper grounding
* Secure I2C wiring

---

## 3. Firmware Upload and Wi-Fi Setup

Before uploading the code:

1. Open the firmware file.
2. Modify the following lines with your Wi-Fi credentials:

```cpp
const char* ssid = "your_wifi_name";
const char* password = "your_wifi_password";
```

3. Upload the code to the ESP32 using Arduino IDE or PlatformIO.
4. Open the Serial Monitor.
5. Wait for Wi-Fi connection confirmation.
6. The ESP32 will display its assigned IP address.
7. Enter that IP address into a browser.

This loads the robot’s control dashboard.

---

# Mathematical Modeling and System Theory

The robot behaves as an inverted pendulum mounted on a wheeled base.

The nonlinear equations of motion are:

Cart translation:

[
(M + m)\ddot{x} + mL \cos\theta , \ddot{\theta} - mL\dot{\theta}^2 \sin\theta = F(x)
]

Pendulum rotation:

[
mL \cos\theta , \ddot{x} + (I + mL^2)\ddot{\theta} - mgL\sin\theta = d(\theta)
]

These equations are nonlinear due to:

* ( \sin\theta )
* ( \cos\theta )
* Coupling terms
* Quadratic velocity components

Direct control design from nonlinear equations is complex. Therefore, the system is linearized around the upright equilibrium:

[
\theta \approx 0
]

Using:

[
\sin\theta \approx \theta
]
[
\cos\theta \approx 1
]

Assuming negligible cart acceleration, the equation reduces to:

[
(I + mL^2)\ddot{\theta} - mgL\theta = d(\theta)
]

Taking the Laplace transform:

[
\frac{\theta(s)}{D(s)} =
\frac{1}{s^2 - a}
]

Where:

[
a = \frac{mgL}{I + mL^2}
]

This transfer function reveals a right-half-plane pole:

[
s = \pm \sqrt{a}
]

The positive pole proves the system is unstable.

This modeling is essential because:

* It explains why the robot falls without control
* It determines required damping
* It guides controller structure
* It validates control choices analytically

---

# Control Strategy

To stabilize the system, a PID controller is implemented:

[
C(s) = K_p + K_d s + \frac{K_i}{s}
]

The PID controller is chosen because:

* Proportional term controls responsiveness
* Derivative term adds damping
* Integral term eliminates steady-state tilt error

The closed-loop characteristic equation becomes third-order, and stability is verified analytically using Routh-Hurwitz criteria.

---

# Embedded Control Architecture

The ESP32 firmware implements two cascaded control loops.

---

## 1. Balance PID (Main Loop)

From the code:

```cpp
float term_bP = Kp * balanceError;
float term_bI = Ki * sumBalanceError;
float term_bD = Kd * (balanceError - lastBalanceError);
```

This loop:

* Stabilizes tilt angle
* Generates motor speed command
* Includes integral windup prevention
* Includes deadband logic
* Includes acceleration limiting

---

## 2. Speed / Drift PID

```cpp
float term_sP = speedOutput * Kp_s;
float term_sI = sumSpeedError * Ki_s;
```

This loop:

* Corrects slow drift
* Modifies desired tilt angle
* Improves long-term stability

The overall structure becomes:

Speed PID → adjusts desired angle
Balance PID → drives motors

This cascaded architecture improves performance and robustness.

---

# Real-Time Web Dashboard

The ESP32 hosts a built-in web server.

When accessing the robot’s IP address, a control interface appears that allows:

* Real-time adjustment of Kp, Ki, Kd
* Adjustment of Kp_s and Ki_s
* Setpoint modification
* Live telemetry display
* Graph visualization
* Motor command controls
* Profile saving and loading

The dashboard includes:

* Live angle measurement
* Live motor speed
* PID term visualization (P, I, D components)
* Historical plotting
* Animated robot visualization

All parameter changes are applied instantly without re-uploading firmware.

---

# Real-Time Tuning

Real-time tuning allows:

* Immediate feedback on parameter changes
* Faster controller optimization
* Practical understanding of damping and overshoot
* Safe experimentation

Profiles can be saved and loaded directly from ESP32 non-volatile memory, enabling multiple tuning configurations.

---

# Repository Contents

This repository includes:

* MATLAB validation scripts
* ESP32 firmware
* Embedded web dashboard
* 3D CAD design files
* PCB layout files

---

# Conclusion

This project demonstrates the complete process of transforming a theoretical unstable nonlinear system into a physically stabilized embedded control system.

It integrates:

* Control theory
* Real-time embedded programming
* Mechanical design
* Electronics design
* Web-based system interaction

The result is a fully functional self-balancing robot with real-time tunable control parameters.

* A version optimized for recruiters
* Or help you format it exactly in GitHub Markdown with sections and spacing perfectly adjusted.

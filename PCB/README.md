## PCB Versions

### Version 1 (V1) - Prototype
This version represents the initial design of the board, characterized by the following specifications and limitations:

* **Microcontroller:** The board only supports the 38-pin ESP32 module.
* **Power:** The power path is shared between the step-down converter and the ESP32 without isolation, preventing USB connection while running on an external battery.
* **Drivers Logic:** Operates at a fixed 3.3V logic voltage (may not be compatible with some clone A4988 drivers that require 5V, unless tracks are manually modified).
* **Control:** Microstepping and Enable settings are hardwired (manually set) and cannot be controlled via code.
* **Installation Notes:**
    * Installing headers requires connecting only the used pins to avoid restart loop issues.
    * The sensor port footprint is reversed by 180 degrees.

---

### Version 2 (V2) - Improved Version
This version was developed to include comprehensive improvements in power management, compatibility, and pin selection:

* **Controller Support:** The board supports mounting ESP32 modules of two different sizes: **30-pin** and **38-pin**.
* **Power Management:**
    * **Isolation Switch:** Allows disconnecting the step-down output from the ESP32, enabling safe chip programming via USB without disconnecting the external power source.
    * **Logic Voltage Selection:** A switch to select the drivers' logic voltage between **3.3V** and **5V** to ensure compatibility with all driver types.
* **Software Control:** Control pins (MS1, MS2, MS3) and the Enable pin are connected to the microcontroller (GPIO) to allow full software control.
* **Pin Selection Optimization:** Avoided using **GPIO 2** for driver control to prevent booting issues occasionally caused by this pin.
* **Extras:**
    * **Voltage Sensor:** Integrated circuit to read input voltage (up to **17V** max).
    * **Track Correction:** The sensor ports' orientation has been corrected to the standard position.

---

### ⚠️ Important Notes
* This design (V2) is still experimental and has not been practically tested yet (Untested).

* Due to the change in the Pinout for this version (specifically avoiding GPIO 2), the Pin Definitions in the software code must be updated to match the new connections before operation.


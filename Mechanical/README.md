## Mechanical Design & 3D Printing

The mechanical structure of the robot was designed and manufactured using 3D printing technology, focusing on a balance between durability and lightweight construction to ensure high responsiveness from the stepper motors.

### 1. Printing Material
The entire chassis was printed using **PLA+ (Polylactic Acid Plus)**. This material was specifically chosen for several technical reasons:
* **Higher Stiffness:** Enhanced rigidity compared to standard PLA, reducing structural flex during rapid balancing movements.
* **Impact Resistance:** Better durability during PID tuning phases where the robot might tip or fall.
* **Dimensional Accuracy:** Ensures a precise fit for motors, bearings, and sensor mounts.

### 2. Slicing Parameters
Infill percentages were optimized based on the mechanical load and stress requirements of each component:

| Component | Infill Percentage | Infill Pattern |
| :--- | :---: | :---: |
| **Wheels** | **50%** | Gyroid / Grid | 
| **Main Body** | **30%** | Tri-Hexagonal | 

<img width="1097" height="1079" alt="image" src="https://github.com/user-attachments/assets/0e2f2c58-1449-4762-8bf1-7ba3b5b0afe2" />

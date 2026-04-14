# RODO - 12-DOF Quadruped Robot

![RODO Robot Photo](doc/Rodo.jpeg)

## 📖 Project Overview

RODO is a custom-designed, 12-Degree-of-Freedom (DOF) quadruped robot developed as a Mechatronics Lab project. The system bridges theoretical kinematics and practical robotics, featuring a fully custom 3D-printed chassis, custom-derived Inverse Kinematics (IK), and a MicroPython-based control architecture running on a Raspberry Pi Pico.

While initial prototypes relied on hardcoded sequences for demonstration, the current codebase features a fully non-blocking trajectory planner, dynamic IK solving, and a synchronised Trot Gait capable of skid-steering (tank turns) and forward/reverse locomotion.

## 🛠️ Hardware & Bill of Materials

* **Microcontroller:** Raspberry Pi Pico
* **Actuators:** 12x MG995 $180^{\circ}$ Servos (3 per leg in a Roll-Pitch-Pitch configuration)
* **Power Regulation:** 5V 12A DC-DC Buck Converter
* **Power Source:** 2x 18650 Li-ion Batteries (~3.7V each) in a 2-cell configuration with a main power switch
* **Chassis:** Fully custom 3D-printed body, thighs, and calf links

## 🧠 Kinematics & Control System

RODO's locomotion is governed by a layered computational pipeline:

1.  **Trajectory Generator:** Uses a non-blocking, time-based mathematical model to generate Cartesian $(X, Z)$ coordinate pairs. The stride consists of a semicircular **swing phase** (in the air) and a linear **stance phase** (pushing against the ground).
2.  **Inverse Kinematics (IK) Solver:** Solves for the thigh ($\theta_2$) and knee ($\theta_3$) joint angles required to reach the target $(X, Z)$ coordinate. Uses the Law of Cosines based on the physical link lengths (L2 = 70.0mm, L3 = 70mm).
3.  **Gait Controller:** Implements a **Trot Gait** by grouping the legs into diagonal pairs (Front-Left & Back-Right vs. Front-Right & Back-Left). A phase offset of 0.5 (50%) between the pairs produces smooth walking.
4.  **Hardware Abstraction:** A custom Servo class translates the mathematical angles into precise 50Hz PWM duty cycles, handling mechanical inversion for mirrored leg assemblies natively.

## 🏗️ Software Architecture

The Python codebase is heavily object-oriented to ensure modularity:

* `Servo`: Handles low-level PWM generation and angle mapping, including an `invert` flag for mirrored legs.
* `Leg`: Manages the specific physical dimensions and IK calculations for a 3-DOF limb.
* `StepGenerator`: A non-blocking time-tracker that calculates the exact required foot position based on elapsed milliseconds, cycle time, and phase offsets.
* `QuadrupedController`: The master orchestrator that commands all four `StepGenerator` instances simultaneously, enabling simple high-level commands like `.set_direction(forward=True)` or `.turn_left()`.

## 🔌 Pin Mapping Configuration

*Adjust this table according to your final physical wiring.*

| Leg | Position | S1 (Roll) Pin | S2 (Thigh Pitch) Pin | S3 (Knee Pitch) Pin | Invert S2 | Invert S3 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Leg 1** | Front Left | 1 | 2 | 3 | False | True |
| **Leg 2** | Front Right | 4 | 5 | 6 | True | False |
| **Leg 3** | Back Left | 7 | 8 | 9 | False | True |
| **Leg 4** | Back Right | 10 | 11 | 12 | True | False |

*(Note: Right-side legs are typically structurally mirrored, requiring the software inversion flags shown above to ensure uniform forward motion).*

## 🔮 Future Improvements

* **3D Kinematic Turning:** Unlocking the S1 (Roll) joint dynamically to allow smooth, scuff-free 3D arc turning rather than skid-steering.
* **Actuator Linkage Modelling:** Adapting the IK math to fully account for the 4-bar mechanical linkage if servos are not mounted directly on the joint axes.
* **IMU Integration:** Adding a 6-axis gyroscope/accelerometer to provide closed-loop feedback for auto-balancing on uneven terrain.


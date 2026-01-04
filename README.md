# 3-DOF Motion Platform Control System

This repository contains the software developed for a **3 degrees-of-freedom (3-DOF) motion platform** intended for **sim-racing applications**.  
The work was carried out as a **9th semester MSc project at Aalborg University**, in collaboration with **Asetek**.

The project focuses on converting real-time sim-racing telemetry data into physically plausible platform motion using classical motion cueing techniques and kinematic modeling.

---

## Project Overview

Sim-racing simulators generate high-frequency and low-frequency motion cues such as accelerations and angular rates.  
Directly reproducing these signals on a physical motion platform is neither feasible nor desirable due to workspace limitations and human perception constraints.

This project implements:

- A **classical Motion Cueing Algorithm (MCA)** to separate low- and high-frequency motion components
- **Tilt coordination** for sustained acceleration cues
- **Inverse kinematics** to map desired platform pose to actuator motion
- Simulation-based validation using recorded telemetry data

The system is designed for analysis, experimentation, and validation rather than as a commercial motion controller.

---

## Degrees of Freedom

The motion platform supports the following degrees of freedom:

- **Heave** (vertical translation)
- **Roll** (rotation about the longitudinal axis)
- **Pitch** (rotation about the lateral axis)

Yaw motion is intentionally excluded due to the intended external controller only being capable of controlling 4 actuators simultaneously.

---
## GitHub structure

In the folder **Matlab scripts** the full simulation of the Motion-Cueing-Algorithm with the Inverse Kinematics Block can be found as well as the recorded telemetry data and a script to plot those.

When running the ControlSystem.slx file, run SimParam.m before to load the simulation parameters into the workspace.

In the folder **Platform Animation** different python scripts can be found for both platform animation, telemetry data animation and a side-by-side animation of platform and data.
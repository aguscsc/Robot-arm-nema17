# 3-DOF Articulated Robotic Arm

**Author:** Agustín Torres  
**Background:** Electronics Engineering Student, Universidad de Concepción (UdeC)
**Status:** Active Development (Phase 1: Kinematic Modeling)

## Project Overview
This repository documents the design, simulation, and physical construction of a custom 3-DOF (Degrees of Freedom) articulated robotic arm. Built with an RRR (Yaw-Pitch-Pitch) configuration, the mechanical architecture mirrors industry-standard robotic arms used in heavy automation and mining applications. 

The primary focus of this project is the rigorous application of control systems and robotics theory, transitioning from pure mathematical models to dynamic simulations, and finally to embedded hardware deployment.

## Hardware Architecture
* **Microcontroller:** ESP32
* **Actuators:** 3x Nema 17 Stepper Motors (Base, Shoulder, Elbow) + 1x Servomotor MG996R (End-effector/Claw)
* **Drivers:** A4988 
* **Chassis:** 3D printed (PLA)

## Software & Control Pipeline

1.  **Kinematic Prototyping (Python):**
    * Derivation of Forward and Inverse Kinematics (IK) using geometric and algebraic (Denavit-Hartenberg) approaches.
    * Workspace plotting and trajectory generation using NumPy and Matplotlib.
2.  **Dynamic Simulation (Simulink):**
    * Physics simulation incorporating the mass and inertia of the 3D-printed links.
    * Design and mathematical tuning of control loops (PID) to ensure stable motion profiles and prevent motor stalling.
3.  **Hardware Deployment (C++ / ESP32):**
    * Translation of simulated control logic into real-time step generation for the motor drivers.
    * Handling of physical constraints, serial communication, and edge-case safety stops.

## Repository Structure
* `/kinematics` - Python scripts for IK solvers and workspace visualization.
* `/simulation` - Simulink models and control system block diagrams.
* `/firmware` - ESP32 C++ codebase for hardware execution.
* `/cad` - STL files and 3D models for the physical build.

## Future Scope
Once the baseline physical model is operational, planned expansions include exploring Hardware-in-the-Loop (HIL) testing and advanced non-linear control algorithms to further optimize the arm's dynamic response.

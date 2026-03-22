# 3-DOF Articulated Robotic Arm

**Author:** Agustín Torres  
**Background:** Electronics Engineering Student, Universidad de Concepción (UdeC)

**Status:** Active Development (Phase 2: Dynamic simulation)

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


# Kinematics 

First, a rough sketch of the arm was drawn to use as a base when writing the kinematic equations. In this sketch $L_0$ $L_1$ and $L_2$ are defined as the lengths of the members for the arm.
![arm sketch](pics/arm_sketch.jpg)

From this the forward and inverse kinematics are defined.

### Forward Kinematics
Forward kinematics allow us to determine the precise $(X, Y, Z)$ coordinate of the end effector given the current angles of the three joints $(\theta_1, \theta_2, \theta_3)$. 

By projecting the arm's geometry onto the horizontal plane and calculating the vertical offsets, we define the position as:

$$X = (L_1 \cos(\theta_2) + L_2 \cos(\theta_2 + \theta_3)) \cos(\theta_1)$$
$$Y = (L_1 \cos(\theta_2) + L_2 \cos(\theta_2 + \theta_3)) \sin(\theta_1)$$
$$Z = L_0 + L_1 \sin(\theta_2) + L_2 \sin(\theta_2 + \theta_3)$$

---

### Inverse Kinematics
Inverse kinematics calculate the required joint angles $(\theta_1, \theta_2, \theta_3)$ needed to reach a specific target coordinate $(X, Y, Z)$. 

**1. Base Rotation ($\theta_1$)**
The base angle is calculated by isolating the $X$ and $Y$ coordinates on the horizontal plane:
$$\theta_1 = \text{atan2}(Y, X)$$

**2. 2D Plane Mapping**
To solve for the shoulder and elbow, we map the 3D target into a 2D side-view plane. We calculate the horizontal distance ($r$), adjust for the base height ($Z_{offset}$), and find the direct line-of-sight distance from the shoulder to the target ($D$):
$$r = \sqrt{X^2 + Y^2}$$
$$Z_{offset} = Z - L_0$$
$$D = \sqrt{r^2 + Z_{offset}^2}$$

**3. Shoulder Angle ($\theta_2$)**
The shoulder angle requires finding the angle of elevation to the target ($\alpha$) and the internal angle of the arm's geometry using the Law of Cosines ($\beta$). We use the standard "Elbow Up" configuration to keep the arm clear of the workspace:
$$\alpha = \text{atan2}(Z_{offset}, r)$$

$$\beta = \arccos\left(\frac{L_1^2 + D^2 - L_2^2}{2 L_1 D}\right)$$

$$\theta_2 = \alpha + \beta$$

**4. Elbow Angle ($\theta_3$)**
The elbow angle is derived by finding the internal angle ($\gamma$) using the Law of Cosines. To get the physical motor angle relative to the extended bicep, we subtract the internal angle from $180^\circ$ ($\pi$ radians):

$$\gamma = \arccos\left(\frac{L_1^2 + L_2^2 - D^2}{2 L_1 L_2}\right)$$

$$\theta_3 = -(\pi - \gamma)$$

### Python simulation

Once the kinematics were properly defined, using a python script it is possible to check if the equations previously mentioned result in the arm reaching the desired target. In this gif obtained using the *ik.py* script the trajectory mapped by the IK (inverse kinematics) is shown.

![kinematics simulation](kinematics/code/ik_simulation.gif)

This script is available in [kinematics/code/](kinematics/code/)
### How to run the simulation

```
git clone https://github.com/aguscsc/Robot-arm-nema17
cd Robot-arm-nema17/kinematics/code
python ik.py
```
You'll be prompted for the coordinates of the target in this format
```
$ python ik.py 
do you want to record the movement? Y(1) / N(0) 0
Insert target coordinates (x y z separated by spaces): 10 10 10
```
If you choose to record the movement to the target provided a *.gif* file with the name **ik_simulation.gif** will be created

---
# Dynamics Simulation (TO DO)

---

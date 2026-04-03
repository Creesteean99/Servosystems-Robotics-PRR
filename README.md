# Servosystems and Robotics Project - PRR Manipulator

## Introduction
[cite_start]This project involves the modeling, simulation, and control of an industrial manipulator as part of the Servosystems & Robotics course[cite: 1, 4]. [cite_start]The primary objective is to execute a complex trajectory approximating the letter "b" located on an inclined plane[cite: 23, 25, 33, 34]. [cite_start]The study covers the entire robotics pipeline, from kinematic modeling to dynamic control and simulation validation[cite: 6, 8, 10, 11].

## Kinematic Analysis
[cite_start]The kinematic study includes both direct and inverse analysis at the position, velocity, and acceleration levels[cite: 6].
* [cite_start]**Direct Kinematics:** Computation of the end-effector position and orientation based on joint coordinates[cite: 6, 13].
* [cite_start]**Inverse Kinematics:** Determination of joint variables required to reach a specific target in the workspace[cite: 6].
* [cite_start]**Singularity Analysis:** Identification of singular configurations where the manipulator loses degrees of freedom or requires infinite joint velocities[cite: 7].
* [cite_start]**Workspace:** Determination of the operational reach according to ISO 9946 standards[cite: 9].

| Coordinate System & Angles | DH Frames & Reference Systems |
| :---: | :---: |
| ![Kinematic Angles](images/schema_cinematica.png) | ![Kinematic Frames](images/schema_cinematica_frames.png) |
| *Structural scheme with joint angles* | *Assignment of coordinate frames* |

## Dynamic Analysis
[cite_start]The inverse dynamic analysis evaluates the forces and torques required by the actuators to perform the assigned movement[cite: 8]. [cite_start]This analysis accounts for the inertial properties, velocities, and accelerations of the manipulator links[cite: 14].

![Dynamic Simulation Graphs](images/coppie_forze.png)

## Control Structure
[cite_start]A control architecture was designed to ensure the manipulator follows the prescribed trajectory while respecting motor limits[cite: 10, 11].
* [cite_start]**Trajectory Planning:** The path uses a lines-parabolas algorithm for the letter "b" and a cycloidal acceleration profile for transitions[cite: 24, 26].
* [cite_start]**Controller Choice:** Implementation of a decentralized or centralized controller (e.g., inverse dynamics or precomputed torques) to minimize the error between theoretical and simulated motion[cite: 10, 15].

![Simscape Controller Scheme](images/schemaControlloreSimscape.png)
![Simscape Control Logic](images/schemaControlloSimscape.png)
![Simscape Cascade Control](images/schemaCascataSimscape.png)

## Results and Simulation
[cite_start]The simulation verifies the continuity of position and velocity throughout the task[cite: 11].
* [cite_start]**Path Execution:** The manipulator moves from $P_1$ to $P_2$, traces the letter "b", and returns to $P_1$[cite: 21, 22, 23, 26].
* [cite_start]**Validation:** Comparison between theoretical trajectories and simulated results to assess control effectiveness[cite: 15].
* [cite_start]**Data Visualization:** Graphs include joint coordinates, motor torques, and end-effector xyz trajectories versus time[cite: 14, 16].

![Final Simulation Result](images/simulazione_simscape_coppie.png)

## Software
* [cite_start]Analytical and numerical methods[cite: 17].
* [cite_start]MATLAB / Simscape[cite: 17].

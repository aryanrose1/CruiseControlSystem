# Automotive Cruise Control System

This repository features an **automotive cruise control system** using **MATLAB** and **Simulink**. A **DC motor** regulates vehicle speed, controlled by a **PID controller** to maintain a set velocity while handling disturbances like load changes or road slopes.

## Overview
- **MATLAB Script (`MATLAB_Script.m`)**  
  - Defines system parameters (motor, vehicle, tachometer).  
  - Implements a **PID controller** for speed regulation.  
  - Performs **root locus, Bode plots, and step response analysis**.  
  - Prepares parameters for Simulink simulation.  

- **Simulink Model (`CruiseControlMod.slx`)**  
  - Models **motor + vehicle dynamics** and **tachometer feedback**.  
  - Uses a **PID controller block** for speed control.  
  - Introduces a **disturbance input** to test system robustness.  
  - Runs simulations and logs angular velocity data.  

## Quick Start
1. **Clone the repository** and open **MATLAB**.  
2. **Run the MATLAB script**:  
   ```matlab
   run('MATLAB_Script.m')

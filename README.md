# 6121C — VEX Robotics Competition Code

Competition robot code for VEX Robotics Team **6121C**, written in C++ for the PROS kernel.

## Overview
This repository holds the autonomous routines and driver-control logic for our competition robot, built on the [EZ-Template](https://ez-robotics.github.io/EZ-Template/) PID/odometry chassis library.

## Structure
- `src/main.cpp` — entry point, driver control, and initialization
- `src/autons.cpp` — autonomous routine definitions (PID-driven drive, turn, and swing motions tuned for our chassis)
- `include/` — header files and subsystem declarations
- `project.pros`, `Makefile`, `common.mk` — PROS build configuration

## Tech
- **Language:** C++
- **Framework:** [PROS](https://pros.cs.purdue.edu/) (VEX V5 open-source kernel)
- **Library:** EZ-Template for PID-based chassis control and odometry

## Notes
PID constants in `autons.cpp` are tuned specifically for our chassis and drivetrain; they'll need retuning for a different robot.

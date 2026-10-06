# 6121C Competition Code

C++ competition software for VEX Team 6121C's 2025–26 Push Back robot, built on PROS and EZ-Template. The robot uses feedback-controlled motion, distance sensing, and coordinated scoring mechanisms to run autonomous match and skills routines.

**Peak #34 globally in Robot Skills · 20 total competition awards**

## Highlights

- **Closed-loop autonomous control:** tuned separate PID settings for linear drive, heading correction, in-place turns, and swing turns, with motion-specific slew limits and exit/settling conditions. The configuration also includes odometry gains used by the library's motion examples.
- **Autonomous repeatability:** reset PID targets, IMU heading, drivetrain sensors, and robot pose before each run, then used hold braking to resist unintended movement. Included interference examples demonstrate recovery resets after obstructed motion.
- **Field-relative correction:** used a forward distance sensor to re-anchor approaches against known field geometry during longer routines, reducing error accumulated from prior movements.
- **Motion sequencing:** changed speed during movement, triggered mechanisms at intermediate distance targets, and chained drive/turn segments to carry momentum without waiting for full settling.
- **Mechanism coordination:** operated dual intake motors and pneumatic scoring/descore systems alongside drivetrain motion, with optical-sensing helpers and a separately scheduled intake-recovery task.
- **Competition performance:** reached a **peak #34 global Robot Skills ranking** during the 2025–26 season, with **20 total competition awards**, including **5× Tournament Champion** and **8× Robot Skills Champion** finishes, plus Excellence, Finalist, Build, and Think awards. [Full breakdown below.](#competition-performance)

## Autonomous control

During autonomous, the robot must drive, collect objects, and score without driver input. Each routine coordinates movement with intake and pneumatic actions. The challenge is finishing within the time limit while keeping turns, approaches, and scoring repeatable.

Before running the selected routine, `autonomous()` in [`src/main.cpp`](src/main.cpp) resets the motion references:

```cpp
chassis.pid_targets_reset();
chassis.drive_imu_reset();
chassis.drive_sensor_reset();
chassis.odom_xyt_set(0_in, 0_in, 0_deg);
chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
```

This clears previous targets, zeros heading and drive measurements, and resets the estimated position and orientation. Hold braking resists movement while stopped. These references depend on the physical starting placement; IMU calibration happens separately during initialization.

## Closed-loop motion

The routines command target distances and headings through EZ-Template rather than treating a motor-power setting and a fixed delay as a reliable position command.

| Primitive | What it controls |
| --- | --- |
| `pid_drive_set` | Forward or backward travel toward a distance target. |
| `pid_turn_set` | An in-place turn toward a heading. |
| `pid_swing_set` | A turn with one side acting as the pivot. |
| `pid_odom_set` | Movement using an estimated position, including coordinate-based paths. |

The controller compares the measured distance or heading with the target and adjusts motor output as the robot moves. Feedback helps it respond to changes in battery voltage and drivetrain load. Wheel slip and contact can still introduce position error.

The tuning in [`default_constants()`](src/autons.cpp) controls four parts of that response:

| Setting | What we tune it for |
| --- | --- |
| Controller gains | Reach the target quickly while limiting overshoot and heading error. |
| Slew limits | Limit initial motor output during acceleration. |
| Exit conditions | Decide how close the robot must be to the target, and for how long, before a movement finishes. |
| Chaining thresholds | Decide when the next movement can begin without waiting for a full stop. |

For example, the drive exit conditions include a 1-inch error band held for 90 ms and a wider 3-inch band held for 250 ms. Chaining uses separate thresholds: 3 inches for driving, 3 degrees for turning, and 5 degrees for swing turns. The integral gains are set to zero, so the configured PID controllers use proportional and derivative terms.

EZ-Template provides the controllers and odometry implementation. Our work is the hardware configuration, tuning, motion sequences, and scoring routines. Competition paths primarily use distance and heading control; the coordinate-based path-following routines included here are library examples. The chassis uses drivetrain sensors and an IMU, without separate tracking wheels enabled.

## Field-relative position correction

Reaching an encoder target does not always mean reaching the intended spot on the field. Wheel slip, contact, and small heading errors can accumulate over a long routine and shift later scoring approaches.

Some routines use the forward-facing distance sensor to calculate the next drive target from the robot's current distance to field geometry. For example, the skills routine includes:

```cpp
chassis.pid_drive_set(((frontDistance.get()) * 0.0393701) - 10, DRIVE_SPEED);
```

The calculation is **measured distance − desired gap**. It converts millimeters to inches, then subtracts ten inches to set the intended gap from the surface. The next drive therefore starts from a fresh field measurement, reducing the effect of earlier forward/backward position error.

This **field-relative correction** re-anchors an approach to the physical field. It requires the sensor to face the intended surface and return a useful reading. It corrects the next drive target rather than reconstructing the robot's full position through sensor fusion.

The included EZ-Template interference example also shows how to recover when movement is obstructed: attempt to back away, then reset drive measurements and retry with a shorter, slower movement if interference continues. This is a library example, separate from the distance corrections in our competition routines.

## Motion timing and chaining

The robot can move quickly between goals, then slow down for a precise scoring approach. Three controls coordinate that timing:

- `pid_wait_until` waits for a distance or heading milestone before issuing the next action. A mechanism can deploy partway through a drive instead of waiting for the entire movement to finish.
- `pid_speed_max_set` changes the active movement's speed limit. Longer approaches can start quickly and slow near a goal or intake interaction.
- `pid_wait_quick_chain` hands off near the target so consecutive movements can blend without waiting for a full settling period each time.

Full `pid_wait()` calls remain where an approach must finish before scoring. Timed delays give game objects time to move through the intake. Motion and mechanism timing are tuned together: a faster drive only helps if the scoring sequence can keep up.

## Game-object handling

The robot uses top and middle intake motors that can run together or at different outputs, plus pneumatic mechanisms for scoring, removing objects from goals, and engaging the match loader. A match loader is a field station used to introduce additional game objects during a run.

[`include/subsystems.hpp`](include/subsystems.hpp) groups the motor, optical sensor, forward distance sensor, and pneumatic definitions. Autonomous routines coordinate these mechanisms with drive commands, so intake motors can keep running while the chassis moves toward the next interaction.

The optical helpers read color hue and proximity to detect game objects. Calibration routines turn on the sensor light and print red and blue hue samples. The separate forward distance sensor measures the field for approach correction.

The `doIntakeUnstuck` helper checks whether the intake is receiving voltage but barely moving. Its recovery branch briefly reverses the intake after a timed stall. A middle-goal routine launches it as a separate `pros::Task`, alongside drive motion. The current helper checks once per invocation; it is not a continuous monitor.

## Autonomous routines and tuning

The codebase contains variants for left and right starting positions, different scoring priorities, autonomous win-point attempts, parking, middle-goal approaches, match loading, and longer skills sequences. An autonomous win point is an additional match-standing reward for completing specified objectives during the autonomous period.

The starting side and alliance strategy determine which routine to use. Routines are registered in `initialize()` for EZ-Template's selector on the robot brain. The current selection list includes `finalSkills` and library test examples; other competition variants are kept in `autons.cpp`.

A controller-triggered distance readout and background position display help diagnose missed approaches and turn errors. The code also retains EZ-Template's optional live PID tuner and controller-triggered autonomous helpers, currently disabled in the driver loop.

That supports a short development cycle: test a segment, adjust its gains, target, speed, or timing, and run it again. Hardware changes also require retuning; the constants are specific to this chassis and its mechanisms.

## Robot Skills

Robot Skills is a separate challenge where one robot scores on the field without an opposing alliance. Rankings combine a driver-controlled skills score with an autonomous programming skills score, so both driving and software affect the result.

Skills routines link collection, loading, scoring, and parking into longer sequences. With more movements per run, repeatable approaches and distance-based corrections become especially valuable. The ranking at the top is the team's season peak, not its final-season placement.

## Competition performance

**20 total competition awards:**

| Award | Count |
| --- | ---: |
| Tournament Champion | 5 |
| Robot Skills Champion | 8 |
| Excellence Award | 2 |
| Tournament Finalist | 3 |
| Build Award | 1 |
| Think Award | 1 |
| **Total** | **20** |

These are team achievements, combining programming, mechanical design, driving, and match strategy. Event records are available through the [6121C RobotEvents profile](https://www.robotevents.com/teams/V5RC/6121C).

## Project structure

```text
src/
  main.cpp        initialization, driver control, autonomous startup/reset logic
  autons.cpp      competition autonomous and skills routines, tuning, examples

include/
  autons.hpp      autonomous routine declarations
  subsystems.hpp motors, sensors, pneumatics, shared hardware definitions

project.pros      PROS project and dependency configuration
Makefile          robot build entry point
```

The repository also contains PROS, EZ-Template, and other bundled framework headers, libraries, and example code. Those are third-party dependencies and starting points, not software authored by our team. The robot-specific work is the hardware configuration, tuning, mechanism control, and competition routine design built on them.

## Built with

- **C++** with [PROS](https://pros.cs.purdue.edu/) on [VEX V5](https://www.vexrobotics.com/v5).
- [EZ-Template](https://ez-robotics.github.io/EZ-Template/) for PID motion control, odometry support, motion chaining, and autonomous selection.
- IMU and drivetrain feedback, distance sensing, optical sensing, motor control, and pneumatics.

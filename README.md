# 6121C Competition Robot

C++ competition software for VEX Team 6121C's 2025–26 Push Back robot, built on PROS and EZ-Template. Combines drivetrain PID tuning, IMU heading feedback, field-relative distance correction, and coordinated motor/pneumatic control across match autonomous and Robot Skills routines.

## Highlights

- **Peak #35 globally in Robot Skills; 20 total competition awards**, including **5× Tournament Champion**, **8× Robot Skills Champion**, **2× Excellence Award**, **3× Tournament Finalist**, **Build Award**, and **Think Award**.
- Tuned separate drive, heading, turn, and swing control loops, with motion-specific gains, settling tolerances, slew limits, and chaining thresholds.
- Recomputed approach distances from forward range measurements to correct accumulated longitudinal error against known field geometry.
- Coordinated scoring actions with distance/heading milestones, adjusted motor-output limits during motion, and chained consecutive movements to reduce settling overhead.
- Sequenced independent intake motors and pneumatic actuators across collection, loading, scoring, and parking routines, with optical hue/proximity sampling and task-based intake-recovery logic.

## Autonomous control

The autonomous period requires the robot to execute without driver input. Each routine coordinates chassis motion and mechanism commands through feedback-based waits, intermediate motion milestones, and timed scoring interactions. The central tuning problem is completing the sequence within the time budget while controlling overshoot, approach alignment, and mechanism timing.

Before dispatching the selected routine, `autonomous()` in [`src/main.cpp`](src/main.cpp) establishes the run's heading and position references:

```cpp
chassis.pid_targets_reset();
chassis.drive_imu_reset();
chassis.drive_sensor_reset();
chassis.odom_xyt_set(0_in, 0_in, 0_deg);
chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
```

This clears prior targets, zeros IMU heading and drivetrain measurements, initializes the estimated pose, and enables hold braking. The reference frame is relative to the robot's physical starting placement; IMU calibration occurs separately during chassis initialization.

## Closed-loop motion

The routines command target distances and headings through EZ-Template rather than treating a motor-power setting and a fixed delay as a reliable position command.

| Primitive | What it controls |
| --- | --- |
| `pid_drive_set` | Forward or backward travel toward a distance target. |
| `pid_turn_set` | An in-place turn toward a heading. |
| `pid_swing_set` | A turn with one side acting as the pivot. |
| `pid_odom_set` | Movement using an estimated position, including coordinate-based paths. |

EZ-Template closes the feedback loops around drivetrain measurements and IMU heading. Our tuning sets the gains and termination behavior for the robot's mechanical response: acceleration, overshoot, heading deviation under load, and settling near a target. Battery voltage and drivetrain load change that response; wheel slip and contact can also introduce measurement error.

[`default_constants()`](src/autons.cpp) defines separate drive, heading, turn, swing, and odometry gains. The configured integral gains are zero, so these PID interfaces currently operate with proportional and derivative terms. Exit conditions combine position or angular error bands with dwell times and timeout limits. For example, drive settling is configured around a 1-inch band for 90 ms and a wider 3-inch band for 250 ms. Chaining thresholds are configured separately: 3 inches for drive, 3 degrees for turns, and 5 degrees for swings.

Those settings control different parts of the motion: gains shape the response, slew settings constrain initial output, exit conditions determine when a movement is considered finished, and chaining thresholds determine when the next movement can take over.

The controller implementation and odometry machinery come from EZ-Template. Our contribution is the hardware configuration, gain and threshold tuning, motion sequencing, and scoring routines. Competition paths primarily use distance and heading control; the included coordinate-based odometry and path-following routines are library examples. The chassis uses drivetrain sensing and an IMU, with separate tracking-wheel declarations disabled.

## Field-relative position correction

Closed-loop convergence to an encoder target does not guarantee the intended field position. Slip, contact, and small heading errors accumulate across a long sequence, shifting later intake and scoring approaches even when individual movements satisfy their exit conditions.

Some routines use the forward-facing distance sensor to calculate the next drive target from the robot's current distance to field geometry. For example, the skills routine includes:

```cpp
chassis.pid_drive_set(((frontDistance.get()) * 0.0393701) - 10, DRIVE_SPEED);
```

The commanded displacement is `measured range − desired standoff`, with millimeters converted to inches. Here, the 10-inch offset sets the nominal separation from the measured surface. Because the displacement is computed from the current range reading, prior longitudinal error is not simply carried into the next fixed-distance command.

This is **field-relative correction**: the next movement depends on the physical field, not just on how far the software thinks the robot has already traveled. Other approaches use different offsets for their geometry. The correction depends on the robot facing the intended surface and receiving a useful reading. It changes a drive target; it is not full sensor fusion or a global pose reconstruction.

The included EZ-Template interference example demonstrates a separate recovery pattern. After a movement reports interference, it attempts to back away; repeated interference causes a drivetrain sensor reset before a shorter, slower movement. This shows how a routine can branch on a failed movement instead of continuing as though it reached its target. That example is distinct from the distance-sensor corrections used in the competition routines.

## Motion scheduling and chaining

The routines coordinate mechanisms against measured motion progress and vary output limits within a segment. This allocates speed to transit while reserving slower, more controlled motion for collection and scoring:

- `pid_wait_until` waits for a distance or heading milestone before issuing the next action. A mechanism can deploy partway through a drive instead of waiting for the entire movement to finish.
- `pid_speed_max_set` changes the active movement's speed limit. Longer approaches can start quickly and slow near a goal or intake interaction.
- `pid_wait_quick_chain` hands off near the target so consecutive movements can blend without waiting for a full settling period each time.

Chaining reduces the time spent settling between compatible movements; full `pid_wait()` calls remain at interactions that need a completed approach. Mechanism delays provide time for object transfer. These are tuned together: shortening a chassis wait is only useful if the intake and scoring sequence can keep up.

## Game-object handling

The robot uses top and middle intake motors that can run together or at different outputs, plus pneumatic mechanisms for scoring, removing objects from goals, and engaging the match loader. A match loader is a field station used to introduce additional game objects during a run.

[`include/subsystems.hpp`](include/subsystems.hpp) groups the motor, optical sensor, forward distance sensor, and pneumatic definitions. Autonomous routines coordinate these mechanisms with drive commands, so intake motors can keep running while the chassis moves toward the next interaction.

The optical-sensing helpers read hue and proximity for object-detection logic. Red and blue sampling routines illuminate the sensor and print measured hue ranges for calibration. These helpers are separate from the forward distance sensor's field-position measurements.

The `doIntakeUnstuck` helper checks for low intake velocity despite applied voltage. Its recovery branch uses a brief reverse command after a timed stall condition. A middle-goal routine launches the helper with `pros::Task`, allowing that work to execute separately from the drivetrain sequence. In this snapshot the helper performs one check per invocation, rather than running as a continuous background monitor.

## Autonomous routines and tuning

The codebase contains variants for left and right starting positions, different scoring priorities, autonomous win-point attempts, parking, middle-goal approaches, match loading, and longer skills sequences. An autonomous win point is an additional match-standing reward for completing specified objectives during the autonomous period.

The routine selected for a match depends on the starting side and the strategy agreed with the drive team and alliance partner. EZ-Template provides the on-brain selector used by `autonomous()`; routines are registered in `initialize()`. The current registration connects the first entry to `finalSkills` and retains the library's test examples. Other competition routines remain in `autons.cpp` for selecting and configuring the desired program.

Between runs, the useful feedback is concrete: how far the approach missed, whether a turn overshot, where an intake stalled, and whether the mechanism finished before the next movement. The program includes a controller-triggered distance readout and a background pose display. It also retains EZ-Template's optional PID-tuning and controller-triggered autonomous helpers, whose call is disabled in the current driver loop.

That supports a short development cycle: test a segment, adjust its gains, target, speed, or timing, and run it again. Hardware changes also require retuning; the constants are specific to this chassis and its mechanisms.

## Robot Skills

Robot Skills is a separate challenge where one robot scores on the field without an opposing alliance. Rankings combine a driver-controlled skills score with an autonomous programming skills score, so both driving and software affect the result.

The skills routines here extend the same motion and mechanism controls into longer sequences: collecting objects, approaching loaders and goals, scoring, repositioning, and parking. With more movements in a run, repeatable approaches and occasional field-relative corrections become especially valuable. The team's season peak is listed above; it is a peak ranking, not a final-season placement.

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

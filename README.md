# 6121C Competition Robot

Competition code for VEX Team 6121C's 2025–26 Push Back robot, written in C++ with PROS and EZ-Template. The work centers on closed-loop autonomous motion, sensor-based position correction, coordinated scoring mechanisms, and tuning routines for repeatable performance on a physical competition field.

## Highlights

- Reached **peak #35 globally in Robot Skills** during the 2025–26 season; team results include **5× Tournament Champion** and **8× Robot Skills Champion**.
- Built and tuned competition routines around PID-controlled driving, turning, and swing turns, with odometry initialization and motion examples for testing.
- Reset controller targets, IMU heading, drivetrain measurements, and estimated pose before autonomous runs to establish a consistent starting state.
- Used a forward distance sensor to re-anchor movements against field geometry instead of relying entirely on accumulated drivetrain measurements.
- Combined mid-motion mechanism actions, speed changes, and motion chaining to balance speed with scoring consistency.
- Integrated independently controlled intake motors and pneumatics, with optical-sensing helpers and an intake-recovery helper launched as a PROS task.

## Autonomous control

VEX matches begin with an autonomous period: the robot must move, collect game objects, and score entirely from code, without driver input. A useful routine needs to do more than reach each target once. It needs to repeat the sequence from the same starting conditions.

Every selected autonomous routine starts through this sequence in [`src/main.cpp`](src/main.cpp):

```cpp
chassis.pid_targets_reset();
chassis.drive_imu_reset();
chassis.drive_sensor_reset();
chassis.odom_xyt_set(0_in, 0_in, 0_deg);
chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
```

These calls clear previous motion targets, establish a zero heading, reset drivetrain measurements, and set the estimated field pose to the origin. Hold braking helps resist unintended movement while stopped. Resetting the IMU heading here establishes a reference; initial sensor calibration happens during chassis initialization.

The physical starting placement still matters. The reset gives the software a controlled reference for that placement rather than carrying measurements from a previous run into the next one.

## Closed-loop motion

The routines command target distances and headings through EZ-Template rather than treating a motor-power setting and a fixed delay as a reliable position command.

| Primitive | What it controls |
| --- | --- |
| `pid_drive_set` | Forward or backward travel toward a distance target. |
| `pid_turn_set` | An in-place turn toward a heading. |
| `pid_swing_set` | A turn with one side acting as the pivot. |
| `pid_odom_set` | Movement using an estimated position, including coordinate-based paths. |

The controller repeatedly compares measurements with the target and adjusts motor output to reduce the error. That feedback matters when battery voltage, drivetrain load, starting alignment, or contact changes how the robot moves. Wheel slip can still corrupt drivetrain-based measurements, so closed-loop motion alone does not eliminate position error.

EZ-Template supplies the controllers and odometry machinery. Our work is the robot-specific configuration, gain tuning, motion targets, speed limits, exit conditions, and sequencing that turn those primitives into scoring routines. [`default_constants()`](src/autons.cpp) keeps the drive, heading, turn, swing, and odometry gains together with acceleration and settling settings.

The competition sequences primarily use distance and heading control. The repository also retains EZ-Template's odometry and path-following examples; those are library demonstrations, not a claim that every match routine uses coordinate-based navigation. The current chassis configuration uses drivetrain sensing and an IMU, with separate tracking-wheel declarations left disabled.

## Position correction and autonomous consistency

A long autonomous sequence can drift even when each individual movement is controlled. A small alignment error early in a run can become a missed intake approach or scoring interaction several movements later.

Some routines use the forward-facing distance sensor to calculate the next drive target from the robot's current distance to field geometry. For example, the skills routine includes:

```cpp
chassis.pid_drive_set(((frontDistance.get()) * 0.0393701) - 10, DRIVE_SPEED);
```

The sensor returns millimeters. Multiplying by `0.0393701` converts the reading to inches; subtracting ten produces a movement target that leaves a nominal ten-inch separation from the measured surface.

This is **field-relative correction**: the next movement depends on the physical field, not just on how far the software thinks the robot has already traveled. Other approaches use different offsets for their geometry. The correction depends on the robot facing the intended surface and receiving a useful reading. It changes a drive target; it is not full sensor fusion or a global pose reconstruction.

The included EZ-Template interference example demonstrates a separate recovery pattern. After a movement reports interference, it attempts to back away; repeated interference causes a drivetrain sensor reset before a shorter, slower movement. This shows how a routine can branch on a failed movement instead of continuing as though it reached its target. That example is distinct from the distance-sensor corrections used in the competition routines.

## Motion timing and chaining

Autonomous time is limited, but running every movement at maximum speed makes close scoring interactions less consistent. The routines use three controls to manage that tradeoff:

- `pid_wait_until` waits for a distance or heading milestone before issuing the next action. A mechanism can deploy partway through a drive instead of waiting for the entire movement to finish.
- `pid_speed_max_set` changes the active movement's speed limit. Longer approaches can start quickly and slow near a goal or intake interaction.
- `pid_wait_quick_chain` hands off near the target so consecutive movements can blend without waiting for a full settling period each time.

The routines still use full waits and mechanism delays where an interaction needs time to finish. The tuning work is deciding where momentum helps and where a controlled stop is worth the time.

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

Team results include:

| Award | Count |
| --- | ---: |
| Tournament Champion | 5 |
| Robot Skills Champion | 8 |
| Excellence Award | 2 |
| Tournament Finalist | 3 |
| Build Award | 1 |
| Think Award | 1 |

These are team achievements, combining programming, mechanical design, driving, and match strategy. Event records are available through the [6121C RobotEvents profile](https://www.robotevents.com/teams/V5RC/6121C).

## My role

[Raahil Russell](https://github.com/RaahilRussell) — **lead programmer and drive-team member**.

My focus was autonomous development and skills programming: tuning motion controllers, adding sensor-based position correction, improving repeatability, and debugging and retuning between matches. I adapted routines as the physical robot changed and worked with the strategy and drive team to choose approaches suited to the starting position and match plan.

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

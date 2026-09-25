# 13765R VEX Robotics Code

Competition firmware for VEX V5 team **13765R**, written in C++17 on the [PROS](https://pros.cs.purdue.edu/) kernel. It was written for the 2023–24 VEX Robotics Competition game **Over Under** (triballs, goals, match-load bars, the elevation bar and the Autonomous Win Point).

One program holds both halves of a match:

- **Autonomous**: a pose tracker (odometry) built on two tracking wheels and an inertial sensor, a set of closed-loop motion primitives (PID turn to heading, turn to face a point, drive to a point with heading correction), and several scripted routines (skills, offensive, two defensive variants).
- **Driver control (op-control)**: tank drive with a pneumatic **power take-off (PTO)**, so the drivetrain runs on either 6 or 4 motors. There are also toggle controls for the intake, flywheel, two wing mechanisms and a PTO-driven hang/lift, and motor-temperature readouts on the brain and the controller.

---

## Contents

1. [Repository layout](#repository-layout)
2. [Building and uploading](#building-and-uploading)
3. [Hardware configuration](#hardware-configuration)
4. [Program lifecycle (PROS competition template)](#program-lifecycle-pros-competition-template)
5. [Power take-off (PTO) model](#power-take-off-pto-model)
6. [Driver control](#driver-control)
7. [Odometry](#odometry)
8. [Motion primitives](#motion-primitives)
9. [Autonomous routines](#autonomous-routines)
10. [Autonomous selector](#autonomous-selector)
11. [Telemetry and display](#telemetry-and-display)
12. [Concurrency model](#concurrency-model)
13. [Tuning reference](#tuning-reference)
14. [Known issues and caveats](#known-issues-and-caveats)

---

## Repository layout

```
.
├── src/
│   ├── main.cpp                   # Competition callbacks, autonomous routines, op-control, UI
│   ├── autonomous_functions.cpp   # Odometry task + motion-control primitives (PID)
│   ├── autonomous_functions.h     # Public API for the above + shared pose globals
│   ├── variables.cpp              # Device construction (ports, gear cartridges, reversal)
│   └── variables.h                # extern declarations for every device
├── include/
│   ├── main.h                     # PROS entry header (includes api.h)
│   ├── api.h, pros/               # PROS 3.8.0 kernel headers
│   ├── okapi/                     # OkapiLib 4.8.0 headers (installed, but not used by src/)
│   └── display/                   # LVGL 5.x headers (used by the PROS LLEMU)
├── firmware/                      # Pre-built static libraries and linker scripts
│   ├── libpros.a, okapilib.a, libc.a, libm.a
│   └── v5.ld, v5-hot.ld, v5-common.ld
├── Makefile, common.mk            # PROS build system
└── project.pros                   # PROS CLI project descriptor (templates, upload slot)
```

All team-written logic is in `src/`. The files under `include/` and `firmware/` are vendor code managed by PROS templates.

---

## Building and uploading

**Toolchain**: PROS CLI with the ARM GCC toolchain it bundles (the PROS VS Code extension works too).

| Template | Version |
|---|---|
| `kernel` | 3.8.0 |
| `okapilib` | 4.8.0 |

```bash
pros make            # compile (default goal "quick": hot/cold linked build)
pros upload          # upload to the V5 brain
pros mu              # make + upload
pros terminal        # open the serial terminal
```

Upload options from `project.pros`:

| Option | Value |
|---|---|
| Program slot | 1 |
| Icon | `clawbot` |
| Description | "General + competition" |
| Project name | `Defensive` |

`USE_PACKAGE:=1` in the `Makefile` turns on **hot/cold linking**. The kernel and libraries go into a *cold* image (`bin/cold.package.bin`) that changes rarely. User code goes into a small *hot* image (`bin/hot.package.bin`). After the first upload, later uploads only resend the hot image, which is much faster.

---

## Hardware configuration

All devices are constructed as globals in `src/variables.cpp` and exposed via `extern` in `src/variables.h`.

### Smart-port devices

| Object | Port | Type | Cartridge | Reversed | Role |
|---|---|---|---|---|---|
| `topLeft` | 8 | Motor | Blue (600 rpm) | yes | Left drive |
| `bottomLeft` | 9 | Motor | Blue | yes | Left drive |
| `topRight` | 1 | Motor | Blue | no | Right drive |
| `bottomRight` | 6 | Motor | Blue | no | Right drive |
| `catapultLeft` | 7 | Motor | Blue | no | PTO motor, left (drive **or** hang) |
| `catapultRight` | 3 | Motor | Blue | yes | PTO motor, right (drive **or** hang) |
| `intake` | 20 | Motor | Blue | no | Triball intake roller |
| `flywheel` | 0 | Motor | Blue | no | Flywheel (see [caveats](#known-issues-and-caveats)) |
| `inertial` | 2 | IMU | – | – | Absolute heading |
| `right_tracker` | 4 | Rotation sensor | – | no | Parallel (forward) tracking wheel |
| `back_tracker` | 10 | Rotation sensor | – | yes | Perpendicular (lateral) tracking wheel |

### Motor groups

| Group | Members |
|---|---|
| `left_motors` | `topLeft`, `bottomLeft` |
| `right_motors` | `topRight`, `bottomRight` |
| `catapult_motors` | `catapultLeft`, `catapultRight` |

The "catapult" names are left over from an earlier version of the robot that had a catapult. On the current robot these two motors are the **PTO motors**. They either help drive the base or run the hang mechanism.

### Three-wire (ADI) pneumatics

| Object | Port | Mechanism |
|---|---|---|
| `wings` | A | Main ("solid") wings |
| `flappy_wings` | B | Secondary ("flappy") wings |
| `PTOpiston` | E | Power take-off shifter |

### Controller

`controller` is the master controller (`CONTROLLER_MASTER`).

---

## Program lifecycle (PROS competition template)

PROS calls these callbacks from `src/main.cpp` according to the field/competition-switch state:

| Callback | Behaviour in this project |
|---|---|
| `initialize()` | Starts the LLEMU (the brain's 8-line LCD emulator), spins until it reports initialized, then sets a black background and white text. |
| `competition_initialize()` | Runs the autonomous selector loop on the three LLEMU buttons until autonomous begins. See [Autonomous selector](#autonomous-selector). |
| `disabled()` | Empty. |
| `autonomous()` | Blocks while `inertial.is_calibrating()`, then calls `auton_skills()`. |
| `opcontrol()` | Starts the telemetry and PTO tasks, then runs the 10 ms driver-control loop. |

---

## Power take-off (PTO) model

The robot has six drive-capable motors. Two of them (`catapultLeft`, `catapultRight`) sit behind a pneumatic PTO controlled by `PTOpiston` on port E.

| `PTOpiston` | `is_PTO_on_base` | Configuration |
|---|---|---|
| `0` (retracted) | `true` | **6-motor drive**: PTO motors mesh with the drivetrain |
| `1` (extended) | `false` | **4-motor drive**: PTO motors drive the hang/lift |

`is_PTO_on_base` is a `std::atomic_bool` declared in `variables.h`. It starts as `true`, so the robot boots in 6-motor drive. Every drive command, in both autonomous and op-control, checks it:

```cpp
left_motors.move(power);
right_motors.move(power);
if (is_PTO_on_base.load()) {        // 6-motor drive
    catapultLeft.move(power);
    catapultRight.move(power);
}
```

This check keeps the PTO motors from being driven as drive motors while they are geared to the hang.

---

## Driver control

`opcontrol()` runs a fixed-period loop with `pros::delay(10)` and keeps a small amount of state for edge-triggered toggles (`get_digital_new_press`).

### Control map

| Input | Action |
|---|---|
| **Left stick Y** | Left side of the tank drive (+ `catapultLeft` in 6-motor mode) |
| **Right stick Y** | Right side of the tank drive (+ `catapultRight` in 6-motor mode) |
| **R1** | Intake: if not currently running forward → forward at full power; if forward → reverse |
| **R2** | Intake: if stopped → reverse; otherwise → stop |
| **L1 tap** (< 750 ms) | Toggle main `wings` |
| **L1 hold** (> 750 ms) | Extend `flappy_wings` while held; on release, the main `wings` open |
| **LEFT** | Toggle flywheel forward at full voltage (12 000 mV) |
| **UP** | Toggle flywheel in reverse (−600 rpm) |
| **A** | In 6-motor mode: shift PTO to 4-motor (hang) mode. In 4-motor mode: toggle hang motors forward at 12 000 mV |
| **X** | In 6-motor mode: shift PTO to 4-motor mode. In 4-motor mode: toggle hang motors in reverse at −12 000 mV |
| **Both sticks X ≤ −110** (push both sticks fully left) | Shift the PTO back to 6-motor drive |

### Drive details

- **Tank drive** with a **±8 deadband** (out of ±127) on each stick. Outside the deadband the raw stick value goes to `Motor_Group::move()`, which maps −127…127 to a voltage. Inside the deadband the side is `brake()`d, which stops a joystick that drifts or sticks slightly off centre from moving the robot.
- The drive block is skipped entirely while `isSwitchingPTO` is `true`, so that the PTO-switching task can take the drivetrain (see below).
- Motors use the PROS default brake mode (`COAST`), so `brake()` removes power and lets the wheels coast.

### Intake state machine

`intakeState ∈ {0 = off, 1 = forward, −1 = reverse}`:

```
            R1                  R1
  off/rev ───────► forward ───────► reverse
     ▲                                 │
     └────────────── R2 ───────────────┘   (R2 from off → reverse)
```

### Wing timing logic

1. On the L1 press edge, `wingTimer = millis()`.
2. While L1 is held and more than 750 ms have passed, `flappy_wings` is extended and `flappy_wingsAreOpen = true`.
3. On release (`wingTimer != 0` and L1 no longer held):
   - After a **long press**, the main wings are extended (`solidWingsAreOpen = true`).
   - After a **short press**, the main wings are toggled.
   - `wingTimer` resets to `0`.

### Flywheel

LEFT and UP share one `flywheelIsMoving` flag. Pressing either button while the flywheel is running stops it. Pressing either button while it is stopped starts it in that button's direction.

### Hang / lift

In 4-motor mode, A and X share the `hangIsMoving` flag and drive `catapult_motors` at ±12 V open loop. When the robot is still in 6-motor mode, the first press of A or X does not run the hang. It extends `PTOpiston` and clears `is_PTO_on_base`.

### PTO switching task

`switch_PTO_state()` is started as a `pros::Task`. It polls `isSwitchingPTO` every 40 ms. When the flag is set, it:

1. Drives the base forward at 200 rpm for 750 ms so the PTO gears move into mesh.
2. Sets `PTOpiston` to the opposite of the current `is_PTO_on_base` state.
3. Clears `isSwitchingPTO`, which gives the drivetrain back to the driver.

Right now nothing sets `isSwitchingPTO` to `true`. The current commit ("Added hang + non-task PTO") shifts the PTO directly from the button handlers instead, so this task never does anything and is kept as infrastructure.

---

## Odometry

`odometry_tracker()` in `src/autonomous_functions.cpp` is a **two-tracking-wheel + IMU** pose estimator. Each routine starts it as a `pros::Task` at the beginning of autonomous. It publishes the robot pose through three `std::atomic<double>` globals:

| Global | Unit | Meaning |
|---|---|---|
| `robot_x` | inches | Field X coordinate |
| `robot_y` | inches | Field Y coordinate |
| `robot_heading` | degrees, [0, 360) | IMU heading (compass convention: 0° = +Y, clockwise positive) |

### Geometry constants

| Constant | Value | Meaning |
|---|---|---|
| `wheel_radius` | 1.375 in | Tracking-wheel radius (2.75 in omni) |
| `sR` | 1.49 in | Lateral offset of the forward (right) tracking wheel from the tracking centre |
| `sB` | 0.512 in | Longitudinal offset of the lateral (back) tracking wheel from the tracking centre |

### Algorithm (per 10 ms iteration)

The sensor data rates are set to 5 ms (`set_data_rate(5)`) so that each 10 ms iteration reads fresh samples.

1. **Wheel arc lengths.** Rotation sensors report centidegrees:

   $$\Delta R = \frac{(\theta_R - \theta_{R,prev})}{100}\cdot\frac{\pi}{180}\cdot r,\qquad \Delta B = \frac{(\theta_B - \theta_{B,prev})}{100}\cdot\frac{\pi}{180}\cdot r$$

2. **Heading change.** The heading comes from the IMU, not from the difference between wheels. $\Delta\theta$ is the signed change from the previous sample, with wrap-around handled so that a move from 358° to 4° gives +6°, not −354°.

3. **Local displacement (arc model).** If $\Delta\theta = 0$, the robot moved in a straight line: $\Delta x_{local} = \Delta B$, $\Delta y_{local} = \Delta R$. Otherwise each wheel is assumed to follow a circular arc about a common instantaneous centre of rotation, and the chord length is:

   $$\Delta x_{local} = 2\sin\!\left(\tfrac{\Delta\theta}{2}\right)\left(\frac{\Delta B}{\Delta\theta} + s_B\right),\qquad \Delta y_{local} = 2\sin\!\left(\tfrac{\Delta\theta}{2}\right)\left(\frac{\Delta R}{\Delta\theta} + s_R\right)$$

   ($\Delta\theta$ in radians.)

4. **Local → global.** The local displacement vector is rotated by the negative of the **average heading** over the interval, $\bar\theta = \theta - \Delta\theta/2$. This is the midpoint approximation of the arc's chord direction. Because heading is clockwise-positive with 0° along +Y, this works out to:

   $$\Delta X = \Delta x\cos\bar\theta + \Delta y\sin\bar\theta,\qquad \Delta Y = -\Delta x\sin\bar\theta + \Delta y\cos\bar\theta$$

5. **Publish.** `robot_x`, `robot_y` and `robot_heading` are written atomically.

The task works on local copies of the pose and only writes the atomics. No other code writes these atomics after the task starts, so it owns the pose.

### Field frame

Coordinates are in inches on the 144 in × 144 in field. Routines set their start pose explicitly (`inertial.set_heading(...)`, `robot_x = ...`, `robot_y = ...`) before starting the odometry task. Several start poses still have a `//Need to measure` comment next to them.

---

## Motion primitives

Declared in `src/autonomous_functions.h`, implemented in `src/autonomous_functions.cpp`. Motor commands use `Motor::move()` (−127…127 → voltage) unless noted otherwise. Values outside that range saturate.

### Heading error convention

Every angular controller uses the same shortest-path error on a circle:

```
if |target − current| ≤ 180:  error = target − current
elif current > target:        error = 360 + target − current
else:                         error = target − current − 360
```

To convert a point to a heading, the code computes `desired = 90° − atan2(Δy, Δx)` (in degrees), normalises it to [0, 360), and adds 180° when `frontFacing == false`, so that the robot backs into the target.

### `robot_set_heading_PID(double angle, bool failSafeIsON = false) → bool`

Turns in place to an absolute heading.

| Parameter | Value |
|---|---|
| kP / kI / kD | 2.53 / 0.31 / 0.51 |
| Loop period | 15 ms |
| Exit condition | \|error\| ≤ 0.8° **and** \|Δerror\| ≤ 0.5°/tick |
| Integral reset | when \|error\| < 0.2° (at target) or \|error\| > 10° (anti-windup: integral only acts in the final 10°) |
| Minimum output | outputs in (1, 7] are raised to 10 and outputs in [−7, −1) to −10, to get past static friction |
| Stall fail-safe | if `failSafeIsON` is set, the turn has run > 800 ms, \|output\| ≥ 7 and the first left motor is below 3 rpm, it calls `stop_robot()` and returns `false` |

It reads heading from `robot_heading`, so **the odometry task must be running** for it to work. The PTO motors get the same differential command when `is_PTO_on_base` is true. It returns `true` when the turn completes normally.

### `robotRotateToPoint(x, y, frontFacing, failSafeIsON = false) → bool`

Works out the heading that points the front (or the back) at `(x, y)` from the current odometry pose, then calls `robot_set_heading_PID`.

### `robotMoveTo(x, y, frontFacing, PIDConstants = 0, initialDirection = 0)`

Drives to a point while correcting heading at the same time. It runs two controllers and mixes them arcade-style:

```
left  = drive + turn
right = drive − turn
```

| Controller | Gains | Error signal |
|---|---|---|
| Drive (PI) | kP = 15, kI = 0.4, kD = 0 | Euclidean distance to the target (inches) |
| Turn (P) | kP set by `PIDConstants` (see below) | Shortest-path heading error to the bearing of the target |

`PIDConstants` picks a turn gain profile:

| `PIDConstants` | turn kP | Use |
|---|---|---|
| 0 (default) | 30 | Standard |
| 1 | 20 | Softer heading correction |
| 2 | 40 | Aggressive |
| 3 | 12 | Gentle, for long sweeping moves |
| 4 | 1 | Almost no heading correction |

Behaviour:

- **Direction.** `direction = ±1` sets the sign of the drive term. It is set from `initialDirection` if that is non-zero, and otherwise from `frontFacing`. Distance error is always non-negative, so overshoot is detected from the error **derivative**: if the error grows by more than 0.2 in/tick while the wheels are still turning in the commanded direction, `direction` is flipped.
- **Integral handling.** Reset when \|error\| < 0.5 in, and cleared whenever the accumulated value exceeds 10 (anti-windup).
- **Turn cut-off.** Within 3 in of the target the turn term is latched to zero. Near the target the bearing to it changes very quickly, and leaving the turn term on would make the robot spin.
- **Exit condition.** Distance ≤ 2 in **and** both sides' first motor ≤ 60 rpm.
- **Stall fail-safe.** If the forward tracking wheel's speed stays under 500 centidegrees/s (≈ 0.12 in/s of travel) for 300 ms, the function returns early.
- Loop period is 15 ms. PTO motors are included in 6-motor mode.

### `robotRotateThenMoveTo(x, y, frontFacing, PIDConstants = 0)`

Composite move: `robotRotateToPoint` first, then `robotMoveTo`. Turning first means the robot starts the drive already facing the target, which helps when the target is far off the current heading.

### `robot_set_velocity(double percent, double milliseconds)`

Open-loop, time-based drive. `percent` is scaled to 600 rpm (the blue cartridge) and applied with `move_velocity()`, which uses the motors' built-in velocity PID. If `milliseconds != 0`, the function blocks for that long and then brakes the drive (and the PTO motors in 6-motor mode). If `milliseconds == 0`, the motors keep running until something else commands them. The routines use this for "ram" moves into goals, and for backing off to "get some space".

### `robotMoveBy(double dist_in_inches)`

A tracking-wheel-only PID for driving a relative distance straight (kP 0.1, kI 0, kD 0.01, targets in centidegrees). **No routine calls it.** See the caveats.

### `robotFollowPoints(double points[])`

Stub reserved for a future pure-pursuit path follower. The body is empty.

### `stop_robot()`

Brakes `left_motors` and `right_motors`.

---

## Autonomous routines

All routines are in `src/main.cpp`. Each one follows the same sequence:

1. Seed the IMU heading and the odometry pose with the starting position.
2. Start `odometry_tracker` as a `pros::Task`.
3. Run a scripted list of motion primitives and mechanism commands.

Coordinates are in inches in the field frame described above.

### `auton_skills()`: Programming Skills (currently active)

Start pose: (36, 16), heading 0°.

1. Open wings. Start the flywheel (`move(100)`).
2. `robotRotateThenMoveTo(8, 32)` using turn profile 4, to reach the match-load bar.
3. Turn to 175° to aim across the field. This is where the robot sits for the **match-loading** phase, with the flywheel launching match loads.
4. Stop the flywheel and close the wings.
5. Back up, turn to 0° and ram the red triballs.
6. With the intake reversed so the robot is not holding a triball, cross the field along y ≈ 11: (36, 11) → (108, 11).
7. Go to the side of the far goal at (132, 36), turn to 0° and ram twice.
8. Move to (84, 36) driving backwards, then (84, 72) at the field centre.
9. Turn to 90°, open the wings and push for 2 s.
10. Back off, point the back of the robot at (80, 84), reverse 800 ms, open the wings and ram again for an angled score.

The routine records `startTime` so that it can time the match-load phase.

### `auton_offensive()`: offensive-side match autonomous

Start pose: (108, 16), heading 0°.

1. Briefly run the intake in reverse (outtake) while driving forward 400 ms, then switch to intaking.
2. `robotMoveTo(76, 65)` (profile 3) to collect a centre triball.
3. Turn to 90°, outtake, open the wings and push both centre triballs into the goal for 1 s.
4. Back off, intake, and collect a triball via (96, 44) → (81, 44).
5. Turn to 90°, outtake with the wings open, then ram twice.

### `auton_defensive_SAFE_AWP()`: defensive side, conservative AWP route

Start pose: (24, 12), heading 135°.

1. Back up, open the wings and drive an uneven arc (L = 80, R = 128) to sweep the match-load-zone triball out of the corner.
2. `robotMoveTo(14, 36)` driving backwards, turn to 0°, outtake, and ram the preload into the goal twice.
3. Back off, then `(36, 12)` → `(60, 12)` with the intake reversed, ending in contact with the elevation bar for the **Autonomous Win Point**.

### `auton_defensive_CENTRAL_SNAG()`: defensive side, centre-triball denial

Start pose: (36, 12), heading 0°.

1. Drive forward, start intaking, and `robotMoveTo(48, 59)` to take a centre triball before the opponent can.
2. Turn away, move to (34, 34), outtake, then go to the match-load bar at (14, 24).
3. Turn to 135°, flick the wings to clear the corner triball.
4. Outtake at full voltage while driving (profile 3) to (63, 10) to touch the elevation bar for the AWP.

---

## Autonomous selector

`competition_initialize()` polls the LLEMU buttons every 5 ms until `pros::competition::is_autonomous()` is true:

| LLEMU button | `auton_state` | Line 0 shows |
|---|---|---|
| Left | `OFFENSIVE` | `OFFENSIVE` |
| Centre | `DEFENSIVE` | `DEFENSIVE` |
| Right | `SKILLS` (default) | `SKILLS` |

> **Note:** `autonomous()` does not read `auton_state` yet. It always calls `auton_skills()`. To run a different routine for a match, change the call in `autonomous()` or add a `switch (auton_state)`. `hasConfimed` is declared but not used.

---

## Telemetry and display

In op-control, a lambda task refreshes both displays every 500 ms:

- **Brain (LLEMU):** `display_all_motor_temps()` prints one line per motor, lines 0–7 in this order: bottom-left, bottom-right, top-left, top-right, intake, PTO-1, PTO-2, flywheel. A line turns **red** when that motor goes above **40 °C**. This matters because V5 motors start limiting current at around 45 °C and cut power further at 55 °C.
- **Controller, line 0:** `display_PTO_state()` shows `6-motor drive` or `4-motor drive`.

`display_hottest_motor()` would show only the hottest motor on the controller, but nothing calls it.

---

## Concurrency model

PROS runs on FreeRTOS. The program uses these tasks:

| Task | Started by | Period | Shared state |
|---|---|---|---|
| `autonomous` / `opcontrol` | PROS kernel | 10–15 ms | – |
| `odometry_tracker` | each autonomous routine | 10 ms | writes `robot_x`, `robot_y`, `robot_heading` |
| display lambda | `opcontrol()` | 500 ms | reads motor temperatures, `is_PTO_on_base` |
| `switch_PTO_state` | `opcontrol()` | 40 ms | reads/writes `isSwitchingPTO`, reads `is_PTO_on_base` |

Every variable shared between tasks is a `std::atomic`, so reads and writes are free of data races without mutexes. The `pros::Task` objects are stack-local, but a PROS 3 `Task` destructor does not delete the underlying RTOS task, so these tasks keep running after the function that created them returns. When the competition mode changes, the kernel ends the mode's own task (`autonomous`/`opcontrol`). Tasks created from inside it are **not** ended automatically. For example, the odometry task keeps running into driver control.

---

## Tuning reference

| Where | Constant | Value |
|---|---|---|
| Odometry | `wheel_radius`, `sR`, `sB` | 1.375, 1.49, 0.512 in |
| Heading PID | kP, kI, kD | 2.53, 0.31, 0.51 |
| Heading PID | settle tolerance | 0.8°, 0.5°/tick |
| Heading PID | min output / fail-safe | ±10 / 800 ms, < 3 rpm |
| Move-to drive | kP, kI, kD | 15, 0.4, 0 |
| Move-to turn | kP by profile 0–4 | 30, 20, 40, 12, 1 |
| Move-to | settle / turn cut-off | 2 in & ≤ 60 rpm / 3 in |
| Move-to | stall fail-safe | < 500 cdeg/s for 300 ms |
| Op-control | stick deadband | ±8 |
| Op-control | wing long-press | 750 ms |
| Telemetry | hot-motor threshold | 40 °C |

---

## Known issues and caveats

These come from reading the current source. The code has **not** been changed; they are listed for the team to look at.

1. **Autonomous selector is ignored.** `autonomous()` always runs `auton_skills()` (see [Autonomous selector](#autonomous-selector)).
2. **Skills match-load wait is skipped.** The wait loop is `while (pros::millis() - startTime > 59000)`. When the loop is reached, far less than 59 s have passed, so the condition is false and the loop exits immediately. The comment ("wait till there are 25 seconds left") suggests `<` and a different threshold were intended.
3. **Flywheel is on port 0.** V5 smart ports are numbered 1–21. PROS rejects port 0 (`errno = ENXIO`), so the flywheel commands in skills and op-control have no effect until the motor is assigned a real port.
4. **`hang` is declared but never defined.** `variables.h` declares `extern pros::Motor hang;` and `variables.cpp` never defines it. The build links only because nothing uses it. The hang actually runs on `catapult_motors`.
5. **`robotMoveTo` does not brake when it returns.** The motors keep their last command, both on normal exit and on a stall-fail-safe exit. The routines usually issue another command straight afterwards, but a final `robotMoveTo` leaves the drive moving (only `auton_offensive` calls `stop_robot()` after one). The stall timer also starts on the first iteration if the robot is still stationary, so the robot has to start moving within 300 ms.
6. **`robot_set_heading_PID` brakes only the four base motors** at the end of a turn. In 6-motor mode the PTO motors keep their last command.
7. **Left-stick deadband is asymmetric in 6-motor mode.** Inside the deadband, `catapultLeft` gets `move(stick value)` while `catapultRight` gets `brake()`. For small stick values the effect is minor.
8. **Flappy wings are never retracted.** After a long L1 press, `flappy_wings` stays extended. There is no `flappy_wings.set_value(0)` anywhere.
9. **`robotMoveBy` is incomplete** (not used anywhere):
   - its target is computed as `goal − current` instead of `current + goal`;
   - `prevError` is read before it is initialised;
   - the loop has no `pros::delay`;
   - the exit test uses signed `derivative > 500` instead of the absolute value;
   - it does not drive the PTO motors.
10. **`display_hottest_motor` has bugs** (not used anywhere):
    - the search starts at index 1, so the label stays empty when motor 0 is the hottest;
    - the label mapping does not match the vector order;
    - it passes a `std::string` to `%s` instead of `.c_str()`.
11. **IMU calibration is implicit.** Nothing calls `inertial.reset()`, so the code relies on the IMU's automatic calibration at power-up. `autonomous()` waits for any calibration in progress. Keep the robot still for about 2–3 s after power-on.
12. **The odometry task is created again in each routine.** Running autonomous more than once without restarting the program (for example from the competition switch during practice) creates a second tracker task that writes the same globals.

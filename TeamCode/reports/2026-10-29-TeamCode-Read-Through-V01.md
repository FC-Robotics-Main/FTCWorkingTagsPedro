# TeamCode Read-Through

Date: 2026-10-29

## Verdict

The folder holds one live match TeleOp (`DuckTapeRobotics_2_1`), which is the older 1.0 TeleOp plus the
"snap to the arc" aiming button. It also holds three Pedro Pathing autonomous programs, two shooter
tuners, the snap math and teach programs, and vendor libraries. (Pedro Pathing is the path-following
library the autonomous programs use to drive.) The snap design report and its figures are here too.

One problem would make the new snap button misbehave on the robot. The Pinpoint settings in
`DuckTapeRobotics_2_1` do not match `pedroPathing\Constants.java`. (The Pinpoint is the goBILDA odometry
computer, the small board that tracks the robot's position from two free-spinning tracking wheels.)
Everything else is either documentation drift or leftover test code.

Nothing was changed and nothing was compiled during this read-through.

## What is in the folder

| File | What it is | State |
|---|---|---|
| `src\...\teamcode\DuckTapeRobotics_2_1.java` | Match TeleOp: mecanum drive, two velocity-controlled shooters (NECTAR = left motor SLM, POLLEN = right motor SRM), intake, 4 servos, Prism LED strips, color sorting, limit-switch ball count, fire gate, saved RPM, and the arc snap on the gamepad1 right stick button | Live; snap untested on the robot |
| `DuckTapeRobotics_1_0.java` | The same TeleOp without the arc snap | Older version, kept as a snapshot |
| `SnapAim.java` | Turns one Limelight result into "distance to the aim point" and "turn needed". Saves and loads the teach file `snap_teach.txt` | Matches report section 5.4 |
| `SnapTeach.java` | Separate TeleOp: on the primary spot, press A to save the taught distance and turn | Matches report section 5.5 |
| `DuckTapeAutoRedPreloadPark.java` | Autonomous "RED L1 Solo: Preload and Park": 4 POLLEN shots from C1, then a 4-leg Pedro path to park by A5 | Main autonomous |
| `newauto.java` | Autonomous "toastauto": a cut-down copy of the one above | Experimental, see problems |
| `DuckTapeAutoPollen4.java` | Autonomous: 4 shots, shake, 2 shots, back off 3 in. | Older; its values have drifted from the TeleOp |
| `ShooterPidfTunerV2.java` | Full shooter PIDF tuner, one shooter at a time, with hit/miss logging | Current tuner |
| `ShooterPIDFTuner.java` | First shooter tuner | Superseded by V2 |
| `MecanumTeleOp.java` | Generic mecanum drive example | Uses wrong motor names, see problems |
| `DriveMotorTest.java` | Runs one drive wheel per button to check wiring | Fine |
| `ShooterLEDControl.java` | Raw I2C writes to the Prism with made-up register values | Placeholder, does not work as written |
| `TelemetryLog.java` | Wrapper that keeps a rolling telemetry history and writes a CSV | Not used by any op mode |
| `pedroPathing\Constants.java` | Pedro drive, follower and Pinpoint settings, with a dated tuning log | Tuning steps 1-8 done; step 9 (centripetal) open |
| `pedroPathing\Tuning.java` | Pedro's stock tuning menu (one local change: drawing on the Panels field) | Vendor file |
| `Prism\*.java` | goBILDA Prism LED driver (MIT licence) | Vendor files, do not modify |
| `2026-10-27-Snap-To-Arc-Aim-Button-V05.md` + `-diagrams\` + `-source\` | Design report for the arc snap, its six figures, and the Python model (`snapmath.py`) behind the numbers | Design only |
| `Claude.Usability.txt` | Personal notes on Claude Code settings and the "adversarial reviewer" prompting pattern | Notes, not code |
| `CLAUDE.md` | Copy of the master workflow rules | Rules file |
| `build.gradle`, `lib\`, `src\main\res\` | Build file (adds the Sloth fast-deploy plugin), annotation processor jar, stock resources | Stock |

## Problems found, most important first

### 1. Pinpoint settings in the TeleOp do not match Constants.java

`DuckTapeRobotics_2_1.java` lines 379-386 still have the placeholder values. `pedroPathing\Constants.java`
lines 89-96 have the measured values, confirmed on the robot on 2026-10-29.

| Setting | TeleOp 2.1 now | Constants.java | What the TeleOp should have |
|---|---|---|---|
| `PINPOINT_X_OFFSET_MM` | 0 | `forwardPodY(-0.177)` in. | -4.5 |
| `PINPOINT_Y_OFFSET_MM` | 0 | `strafePodX(-1.831)` in. | -46.5 |
| `PINPOINT_X_DIR` | FORWARD | `forwardEncoderDirection(REVERSED)` | REVERSED |
| `PINPOINT_Y_DIR` | FORWARD | `strafeEncoderDirection(FORWARD)` | FORWARD |

The X direction matters most. With it backward, the Pinpoint thinks "toward the front" is "toward the
back". Then the snap's drive step sends the robot away from the cell when it should go toward it. The
report's bench check 5 is the test that catches this. The two offsets are small (4.5 mm and 46.5 mm),
but the 46.5 mm one makes position drift while the robot turns in place.

### 2. `newauto.java` ("toastauto") freezes INIT for 10 seconds

- `sleep(10000)` runs before any setup. The Driver Station sits on INIT for 10 s, and the trigger servo
  is not homed during that time.
- `Wait_Time = 10` is declared but never used. A "wait before moving" belongs after Start, not in INIT.
- `BACK_OFF_WALL_IN = 60`, but the header comment still says 3 in. The path is now just one straight
  60 in. move; `parkNudge` is declared and never built or used.

### 3. Op mode copies have drifted from the TeleOp

Every autonomous program says "same as TeleOp", but the values differ:

| Value | TeleOp 2.1 | Preload and Park / toastauto | Pollen4 |
|---|---|---|---|
| POLLEN P | 160 | 160 | 180 |
| POLLEN start RPM | 2400 (TeleOp loads the saved file instead, if one exists) | 2400 | 2200 |
| Fire window | +/-45 | +/-45 (averaged over 6 readings) | +/-60 |

The tuner V2 also starts POLLEN at P = 180. Decide which numbers are the truth before the next event.

### 4. `MecanumTeleOp.java` uses the wrong motor names

It looks for `FL`, `FR`, `BL`, `BR`. The robot configuration uses `FLM`, `FRM`, `BLM`, `BRM`. Selecting
this op mode stops it with a "device not found" error.

### 5. `ShooterLEDControl.java` cannot work as written

It talks to the Prism through raw I2C at register 0, using made-up color codes 1, 2 and 3. It also
needs a device named `prism_i2c`, while the real Prism is configured as `prism`. The TeleOp already
drives the Prism correctly through `GoBildaPrismDriver`.

### 6. Report and code disagree in two places

- **Button.** The V05 report says "hold the right trigger". The built TeleOp uses one press of the
  gamepad1 right stick button, because the right trigger is the intake. The code comments are correct;
  the report is out of date.
- **Aim point.** The report says the aim point is "the middle of the cell floor". `SnapAim.java` says it
  is NOT the middle of the cell floor (that is 133 mm behind the lip); it is the spot under where the
  primary shot passes, 75 mm behind the lip. The math is the same either way. Only the wording differs.

### 7. House style is not followed yet

The project rules ask for beginner-level comments, a WHAT / CALLED FROM block on every method, Hungarian
prefixes on primitives, and if/else instead of the ternary operator. (A ternary is the short
`condition ? a : b` form.) None of the op modes follow these rules yet. Examples:

- `SnapAim` and `SnapTeach` use names like `ll`, `rd`, `r`, `f`, `t`, `st`.
- The TeleOp uses many ternaries.

This is a cleanup job, not a bug.

### 8. Housekeeping

- `TelemetryLog.java` is unused.
- `-source\__pycache__\` is Python build output and does not belong in the repository.
- The design report, its figures and `CLAUDE.md` are not committed to git yet.
- The repository's latest commits have uninformative messages ("test", "test2", "Add files via upload").

## What a green compile would and would not prove

No compile was run, because nothing changed. After problem 1 is fixed, run
`./gradlew :TeamCode:compileDebugJavaWithJavac`. A clean compile only proves the code is valid Java with
the right types. It does not prove the snap drives the right way. That still needs these checks on the
robot, from report section 5.7:

- bench check 4: push the robot by hand and watch the Pinpoint X, Y and heading
- bench check 5: the first powered snap, at 0.25 power

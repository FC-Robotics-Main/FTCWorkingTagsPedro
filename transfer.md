# Transfer / Handoff Notes — SampleAutoTriangleAprilTag

Purpose: hand this work off (e.g., to Fable for a fresh review). It captures WHAT the
code does, the history of how it got here, every naming/style decision, and — most
importantly — what still needs to be verified **on the physical robot** (the compiler
cannot check those).

---

## 1. What this op mode is

An FTC autonomous op mode that combines **Limelight 3A AprilTag tracking** with
**Pedro Pathing** (path following + odometry). High-level flow:

1. **VISION PHASE** — find AprilTag #11 and drive to park ~1 foot in front of it:
   - SCAN: spin slowly until the tag is seen.
   - PRE-ALIGN: turn until facing the tag (TX within ±2°) before driving.
   - SQUARE-UP: strafe sideways onto the tag's centerline so we arrive head-on, not
     at an angle. Uses `getRobotPoseTargetSpace()` (robot pose seen from the tag).
   - APPROACH: drive forward until the tag fills `TARGET_AREA_CLOSE` % of the image (TA),
     steering continuously to stay aimed.
   - PARK: stop when close enough AND aimed.
   - If the tag is lost for >2 s, give up: drive back to where it was lost, then run the
     triangle anyway.
2. **TRIANGLE PHASE** — snap odometry back to a fixed start point **facing 180°**, then
   drive a pre-built equilateral triangle (5 legs).

TX = "Target X" = horizontal angle to the tag in degrees (0 = centered).
TA = "Target Area" = % of the camera image the tag fills (bigger = closer; a distance
stand-in). "Fiducial" = AprilTag. "LL" in class names = Limelight.

## 2. File / versioning scheme

- `SampleAutoTriangleAprilTag.java` — the un-versioned baseline, restored to the last
  committed (original) version.
- `SampleAutoTriangleAprilTagV00.java` — frozen baseline snapshot (= the original).
- `SampleAutoTriangleAprilTagV01.java` — **current working version**; all of the
  readability work below lives here.
- Rule: to change the op mode, copy the highest version to the next number
  (V01 → V02) and edit that. **Never clobber a lower version.** Each version file must
  have its `class` renamed to match the filename and a unique `@Autonomous(name=... VNN)`,
  or the project will not compile / will show duplicate op-mode names.

## 3. Behavior/logic decisions & history (the non-obvious stuff)

- **Distance is measured by TA, not 3D pose.** The Limelight 3D-pose distance
  (`getRobotPoseTargetSpace().getPosition()`) was unreliable / axis-confusing, so the
  stop condition uses TA (target area %) instead.
- **`TARGET_AREA_CLOSE = 55.0`** was MEASURED, not calculated: watch "TA max" on the
  Driver Station; the peak before the tag left view was ~60, so 55 stops just before we
  lose sight (~1 ft). Re-measure if tag size or camera mount changes.
- **Vision phase drives via `pathFollower.startTeleopDrive()` + `setTeleOpDrive()`**,
  NOT direct motor writes. An earlier attempt using direct `DcMotor.setPower()` caused
  the follower's `update()` to fight the motor commands and introduced a turn-direction
  sign flip. Reverting to teleop-drive mode fixed it.
- **The 180° handoff.** Original code rebuilt leg1 from the robot's live pose; that left
  the follower in an uncertain frame and the robot "drove forward forever" (a leg never
  registered complete). Fix: at the triangle handoff, `breakFollowing()` then
  `setPose(new Pose(startX, startY, Math.PI))`. Because leg 1 holds heading 0, the robot
  does a clean 180° then drives the fixed pre-built legs, which complete reliably.
  Pi (π) here just means 180° (headings are in radians).
- **Lost-tag timeout:** first frame the tag goes missing we record the time and pose;
  after 2000 ms still missing we set `bIsReturnNeeded`, drive back to `tagLostPose`, then
  run the triangle.
- **Square-up sign is unverified** (see below).

## 4. MUST VERIFY ON THE ROBOT (compiler can't check these)

- **`STRAFE_GAIN` sign / strafe direction.** If, during SQUARE-UP, the robot strafes the
  WRONG way (away from center), flip the sign of `STRAFE_GAIN`.
- **Is the 3D-pose square-up data trustworthy on this camera?** If `Lateral`/`Yaw`
  telemetry looks jittery/garbage, set `USE_SQUAREUP = false` to fall back to
  point-and-drive.
- **`TARGET_AREA_CLOSE` tuning** for the real "1 foot" stop distance.
- **Triangle orientation** after the 180° reset — confirm it draws the intended shape
  on the field from the parked position.
- **All the `setTeleOpDrive(forward, strafe, turn, robotCentric)` sign conventions** and
  power thresholds (0.06 min turn, 0.10 min drive, etc.).
- **Hardware config names** must match exactly: `front_left_motor`, `front_right_motor`,
  `back_left_motor`, `back_right_motor`, `pinpoint`, `limelight`.

## 5. Coding standards applied (so a reviewer keeps them consistent)

- **Audience is students** — comments must be understandable to a beginner; jargon is
  explained at first use (odometry, pose, heading, radians/π, teleop, strafe, centerline,
  fiducial=AprilTag, LL=Limelight, TX, TA, isBusy, clamp, pipeline).
- **Hungarian prefixes on primitives only:** `b`=boolean (and booleans read as `bIs…`),
  `n`=int/long, `f`=float/double, `str`=String, `c`=char. Team always uses `long` (not
  `int`) and `double` (not `float`), so `n`/`f` never lie.
- **Constants stay `UPPER_SNAKE_CASE`** with NO Hungarian prefix.
- **Descriptive names, never cryptic.** Objects: `pathFollower`, `limelightCamera`,
  `elapsedTimer`, `telemetryLog`. Locals: `latestResult`, `detectedTags`, `targetTag`,
  `tag`, `robotPoseInTagFrame`, `robotPositionInTagFrame`, `currentPose`. Waypoints:
  `triangleTop`, `triangleBottomLeft`, `triangleBottomRight`, `triangleBelowBottomMiddle`.
  Legs named by endpoints: `leg1StartToTop`, `leg2TopToLeft`, `leg3LeftToRight`,
  `leg4RightToTop`, `leg5TopToStart`.
- **No abbreviations in prose** (spell out "Driver Station", not "DS").
- **No ternary operator** in normal code — use `if/else`. The ONLY exception is
  `OPMODE_NAME`, where an annotation requires a compile-time constant expression (an
  if/else is a statement and won't compile there). This is documented at that line.
- **Comment spacing:** a blank line ABOVE a comment block; the comment sits directly on
  top of the code it describes (no gap below); never jam a comment under unrelated code.
- **Every method** has a `WHAT:` / `CALLED FROM:` doc block.
- **Imports** are grouped by function with per-line comments. **Do NOT run Android
  Studio "Optimize Imports"** on the file — it strips the comments and grouping.
- **Op-mode name is built at compile time** from the `RUN_PATHS` / `RUN_APRILTAG` /
  `RUN_LOG_DEMO` feature switches (which double as poor-man's conditional compilation).

## 6. Compile status

`./gradlew :TeamCode:compileDebugJavaWithJavac` → **BUILD SUCCESSFUL**. This compiles the
entire TeamCode source set (baseline + V00 + V01) against the real FTC SDK, so all
renames and edits are syntactically valid and type-correct. It does NOT validate robot
behavior — see section 4.

## 7. Suggested focus for the next reviewer (Fable)

Syntax is already clean (it compiles). Spend effort on **logic and sign conventions**:
the strafe direction, the teleop-drive sign conventions, the phase-transition
conditions, and whether the tuning constants (`TARGET_AREA_CLOSE`, gains, thresholds)
produce the intended motion. Section 4 is the checklist.

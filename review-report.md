# Code Review Report — SampleAutoTriangleAprilTagV01

| | |
|---|---|
| **File reviewed** | `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SampleAutoTriangleAprilTagV01.java` |
| **Reviewed by** | Fable (Claude), pre-first-hardware-run confidence pass |
| **Date** | July 8, 2026 |
| **Baseline** | `SampleAutoTriangleAprilTagV00.java` — read for reference, **not modified** |
| **Compile status** | `./gradlew :TeamCode:compileDebugJavaWithJavac` → **BUILD SUCCESSFUL** after all fixes |

**Scope of the review:** trace the vision-phase state machine end to end, audit every
sign/direction convention, sanity-check the square-up 3D math, review the tuning
constants, and confirm the code meets the project's coding standards (CLAUDE.md).
Syntax and typos were out of scope — the project already compiled cleanly.

---

## Section A — Definite code bugs (found and FIXED)

### A1. 🔴 Triangle-only builds spun 180° and drove the triangle mirrored

**Where:** the triangle-phase one-time setup inside `loop()` (around line 684).

**What was wrong:** With `RUN_APRILTAG = false` (a "Triangle Only" build), `init()`
marks the camera phase as already done, so the first `loop()` goes straight to the
triangle. That setup code unconditionally declared the robot's heading to be
π radians (180°). The 180° declaration is only correct when the camera phase
actually ran and left the robot facing the tag. In a triangle-only build the robot
is physically facing forward at its true starting spot — so the follower would
spin it half-way around and then drive the entire triangle rotated 180° through
the real field. Nothing like what the plain triangle op modes do.

**Why it matters:** flipping the feature switch to run a plain triangle — a likely
classroom demo — would produce baffling wrong motion and look like a robot fault.

**Fix applied:** the declared heading is now chosen with an if/else:
π when `RUN_APRILTAG` is true (the field-tested behavior, unchanged), 0 when the
camera phase is compiled out (matching the plain triangle op modes).

```java
double fTriangleStartHeadingRadians = 0.0;
if (RUN_APRILTAG) {
    fTriangleStartHeadingRadians = Math.PI;
}
pathFollower.setPose(new Pose(
        triangleBelowBottomMiddle.getX(),
        triangleBelowBottomMiddle.getY(),
        fTriangleStartHeadingRadians));
```

---

### A2. 🔴 Square-up could freeze the robot for the rest of the autonomous

**Where:** the SQUARE-UP "waiting for pose" branch in `aprilTagUpdate()`
(around line 570).

**What was wrong:** if the camera's 3D position reading
(`getRobotPoseTargetSpace()`) came back null on **every** frame — a real
possibility, for example if the Limelight pipeline was never configured with the
tag size, which disables 3D solving — the "stop and wait" branch repeated forever:
stop motors, return, next loop, same thing. The robot would sit frozen for the
rest of the autonomous with the tag in plain view. This is a
"phase that cannot be exited."

**Why it matters:** a single camera-configuration mistake would silently turn the
whole autonomous into a parked robot.

**Fix applied:** a 1.5-second cap, using the same stopwatch pattern as the
lost-tag timeout. If no position data arrives within 1.5 seconds, the code logs a
warning, marks square-up as done, and falls through to the plain point-and-drive
approach — which needs only TX and TA and always works. The stopwatch resets
whenever data does arrive, so brief one-frame dropouts do not accumulate.

---

### A3. 🟡 The file header contradicted the actual behavior

**Where:** the big comment block at the top of the file.

**What was wrong:** three separate staleness problems:

1. It still said the robot stops at "≈ 1 foot … at **35%**" — the constant is
   **55**, and it was measured, not calculated.
2. It described a 3-step vision phase (SCAN / ALIGN / APPROACH) — the real machine
   has five steps (SCAN / PRE-ALIGN / SQUARE-UP / APPROACH / PARK).
3. It never mentioned the 2-second lost-tag safety net at all.

**Why it matters:** a student reads the header first and forms the wrong mental
model before ever reaching the code.

**Fix applied:** the header now lists all five steps plus the safety net, and
points at the `TARGET_AREA_CLOSE` comment for the tuning story instead of quoting
a stale number.

---

### A4. 🟢 Coding-standards violations

All fixed:

| Violation | Where | Fix |
|---|---|---|
| Comment jammed against the code above it (no blank line) | "One flag per leg" comment; "Start leg 1 right away" comment | Blank lines added |
| `int` used despite the "always `long`" rule | `TARGET_TAG_ID`; local `nTagsSeen` | Both changed to `long` (the `==` against `getFiducialId()` promotes safely) |

Checked and **clean**: the only ternary in the file is the required `OPMODE_NAME`
one (annotations demand a compile-time constant expression); Hungarian prefixes
are consistent (`bIs…`, `n…`, `f…`, `str…`); constants are `UPPER_SNAKE_CASE`
with no prefix; names are descriptive; every method has a WHAT / CALLED FROM
block; jargon is defined at first use.

---

## Section B — Sign conventions: proven vs. unprovable from code

| Convention | Verdict | Basis |
|---|---|---|
| **Turn** (`-TX * 0.02`) | ✅ CONFIRMED | Pedro's positive turn input is counterclockwise (verified in the extracted Follower library source). Positive TX (tag to the right) → negative turn → clockwise → correct. Also proven by your own field tests ("alignment latched perfectly"). |
| **Forward** (positive = toward tag) | ✅ CONFIRMED | Proven by your field tests — the robot drove to (and under) the tag. |
| **Strafe** (`-lateral × STRAFE_GAIN`) | ⚠️ **50/50 — CANNOT be confirmed from code** | Two unverified links: (a) how the FTC SDK maps Limelight target-space X into `Pose3D`, and (b) which direction Pedro treats as positive strafe. |
| **180° spin direction at triangle start** | ℹ️ Ambiguous by design | With a heading error of exactly 180°, the direction Pedro picks is mathematically arbitrary — it may spin left one run, right another. Harmless; don't let it spook you. |

**Field test for the strafe sign:** place the robot about 2 feet to the LEFT of
the tag's centerline and run the op mode. Watch two things on the Driver Station:
the sign of `Lateral (in)`, and which way the robot strafes. If it strafes AWAY
from the centerline, flip the sign of `STRAFE_GAIN` (the code comment at that
line says exactly this).

---

## Section C — Things to verify ON THE ROBOT (no code changed)

1. **The RETURN path may inherit the old "drove forever" disease.** The lost-tag
   return builds a path from the live post-teleop pose — structurally the same
   pattern that failed before the 180° reset fixed the triangle. It plausibly
   works (both endpoints live in the same odometry frame), but it was never
   field-tested. Watch the `RETURNING to lost pose` phase; if it never finishes,
   the return phase needs the same clean-frame treatment the triangle got.

2. **SCAN has no overall timeout.** If the camera is dead or the tag simply is
   not there, the robot spins in place for the entire autonomous. Not changed,
   because the right timeout (5 s? 10 s? then give up and run the triangle?) is a
   strategy call. It is a five-line fix mirroring the lost-tag pattern when
   wanted.

3. **The triangle is RELATIVE after the vision phase.** The waypoints are
   re-anchored to wherever the robot physically parked. If the tag is near a
   wall, part of the "triangle" may pass through that wall. Walk the expected
   shape from the parked spot before the first full-speed run.

4. **Tuning watch-list:**
   - oscillation around `CENTER_TOLERANCE_INCHES` (minimum strafe power 0.08
     versus the 2.0-inch tolerance band);
   - stalling at the 0.10 minimum drive power as TA approaches 55;
   - whether TA = 55 still lands at roughly 1 foot (re-measure after any tag or
     camera-mount change).

5. **Accepted edge case (no action):** if the robot starts already close and
   centered (TA ≥ 55 and TX ≤ 2°), it parks immediately without squaring up —
   i.e., possibly parked at an angle. Rare and benign; noted so it does not
   surprise anyone.

---

## Section D — Bottom line

Two real bugs fixed (one behavioral regression in triangle-only builds, one
freezable phase), the header brought back in line with the code, standards clean,
and the build is green after every change. The turn and forward signs rest on
your own field evidence and can be trusted. The two things this code **cannot**
prove about itself are the strafe direction and the return path — test those
first, using the procedures in Sections B and C.

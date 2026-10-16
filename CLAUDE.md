# Working preferences for this project

This is an FTC robot project (Java, Pedro Pathing + Limelight AprilTag vision). The
code doubles as **teaching material for students**, so clarity beats cleverness.

## How to work with me

- **Do not ask for permission or confirmation.** Make the sensible decision, do it, and
  tell me what you did afterward so I can correct course. Do not stop to ask on things
  with an obvious default. (Accept-edits mode is on by default in settings.)
- **Show me the change after you make it** — paste the edited block — so I don't have to
  ask to see it.
- **Don't be terse to save space.** Prefer clear, complete comments and explanations.
- When something can't be done (or can only be done one way), say so plainly and explain
  why, ideally with proof (e.g., a quick compile test), instead of guessing.

## Comments — write for a beginner student

- If a coach couldn't understand a comment, a student won't either. Explain jargon the
  FIRST time it appears; don't wait to be asked. Terms to always define in context:
  odometry, pose, heading, radians/π, teleop, strafe, centerline, fiducial (= AprilTag),
  "LL" (= Limelight), TX, TA, isBusy, clamping, pipeline.
- Spell out abbreviations in prose — e.g. "Driver Station", not "DS".
- Every method gets a comment block with **WHAT** it does and **CALLED FROM** where.
- A comment explaining code sits directly on top of that code, with a **blank line above
  the comment**. Never jam a comment against the line above it or under unrelated code.

## Naming

- **Hungarian prefixes on PRIMITIVES only:** `b` = boolean (booleans read as `bIsXxx`),
  `n` = int/long, `f` = float/double, `str` = String, `c` = char.
- We always use `long` (not `int`) and `double` (not `float`), so `n`/`f` never mislead.
- **Constants stay `UPPER_SNAKE_CASE`** with no prefix.
- **Objects/classes: descriptive names, never cryptic.** No single letters, no `rp`/`pos`
  style names. Name things for what they are and do.

## Style

- **Avoid the ternary operator; use `if/else`.** Only exception: where a compile-time
  constant expression is required (e.g., an annotation value), because an if/else is a
  statement and won't compile there. Document that spot.
- Keep lines short enough to print in landscape.
- **Do not run Android Studio "Optimize Imports"** on files whose imports are grouped
  with explanatory comments — it strips them.

## Versioning (op mode files)

- Keep numbered snapshots (`...V00`, `...V01`, ...). To change an op mode, copy the
  highest version to the next number and edit that. **Never clobber a lower version.**
- Each version file must have its `class` renamed to match the filename and a unique
  `@Autonomous(name = "... VNN")`.

## Verifying changes

- After nontrivial changes, compile before declaring success:
  `./gradlew :TeamCode:compileDebugJavaWithJavac`
- Be clear about what a green compile does and does NOT prove: it confirms syntax/types,
  not robot behavior. Call out what still needs on-robot testing.

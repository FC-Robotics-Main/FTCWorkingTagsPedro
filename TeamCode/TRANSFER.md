# Transfer briefing: FTC TeamCode (DuckTape Robotics)

## 1. Project goal
FTC robot code (Java, FTC SDK 11.1, Pedro Pathing 2.1.2, Limelight 3A AprilTag vision). Pedro Pathing
is the library that makes the robot follow planned paths in autonomous. The robot is a mecanum
drivetrain with two flywheel shooters (NECTAR = left motor SLM, POLLEN = right motor SRM), an intake,
four servos and Prism LED strips. Its front is the intake side and its back is the shooter side. The
code doubles as teaching material for students. The current feature in progress is "snap to the arc":
one button aims the robot's back at the cell and sets its distance using the cell's AprilTags.

## 2. Current state
- Match TeleOp: `DuckTapeRobotics_2_1.java`. It is 1.0 plus the arc snap on the gamepad1 right stick
  button. It compiles in principle but is untested on the robot.
- Snap math is in `SnapAim.java`. The teach program is `SnapTeach.java`. Both match the design report
  `2026-10-27-Snap-To-Arc-Aim-Button-V05.md` in this folder.
- Pedro tuning in `pedroPathing\Constants.java` is done through step 8 (drive PIDF). Step 9
  (centripetal) is open. Backward stops overshoot about 5 in., which is parked for now.
- The main autonomous is `DuckTapeAutoRedPreloadPark.java`. `newauto.java` ("toastauto") is experimental.
- The full read-through is in `reports\2026-10-29-TeamCode-Read-Through-V01.md`.

## 3. Key decisions & constraints
- **The project's `CLAUDE.md` files set the rules:**
  - comments written for a beginner student
  - Hungarian prefixes on primitives only
  - if/else instead of the ternary operator
  - versioned op mode snapshots (`V00`, `V01`, ...); never overwrite a lower version
  - archive, never delete
  - compile with `./gradlew :TeamCode:compileDebugJavaWithJavac` after nontrivial changes
- All snap measurements are metric (mm and degrees). Robot directions: FRONT = intake side, BACK =
  shooter side and camera.
- One teach press on the primary spot covers all four cells and both alliances. It cancels errors in
  how the camera is mounted.

## 4. File map
- `src\main\java\org\firstinspires\ftc\teamcode\`: all op modes (see the report's table).
- `pedroPathing\Constants.java`: drive, Pinpoint and follower constants, with a dated tuning log.
- `Prism\`: goBILDA vendor LED driver. Do not modify it.
- `2026-10-27-Snap-To-Arc-Aim-Button-V05*`: the snap design report, its figures and its Python model.
- `reports\`: dated, versioned reports.

## 5. Open items / next steps
1. DONE 2026-10-29: `DuckTapeRobotics_2_2.java` (TeleOp "DuckTapeRobotics-2.2") is 2.1 with the Pinpoint
   settings copied from `Constants.java`: X offset -4.5 mm, Y offset -46.5 mm, X direction REVERSED.
   It compiles. 2.1 is unchanged and still has the wrong values; use 2.2.
2. Do not write report files unless the user asks for one (the user said so on 2026-10-29).
   DONE 2026-10-29: "limelight" added to robot configuration DuckTape1 (it was missing). Limelight
   pipeline 0 fixed in its web page (http://192.168.43.1:5801 on the robot Wi-Fi): ID filter "11"
   cleared, tag size 101.6 -> 82.55 mm, Full 3D on, Detector Downscale 2 -> 1. Read back through the
   hub: tags 30-33 all report 3D positions about 1.44 m away. Downscale 1 dropped the camera from about
   58 to 26 frames a second; still enough for the snap's 5 readings in 0.4 s. Read settings with
   `adb shell 'printf "GET /pipeline-atindex?index=0 HTTP/1.0\r\n\r\n" | toybox nc -w 3 172.29.0.1 5807'`.
   DONE 2026-10-29 17:40: Snap Teach saved on the hub: snap_teach.txt = "1073.56,-2.416" (distance mm,
   turn deg). Tag height and row length checks were good. The user later changed the Limelight to
   exposure 1628 and flicker 50 Hz; flicker should be 60 Hz (US mains).
   DONE 2026-10-29: `DuckTapeRobotics_2_3.java` (TeleOp "DuckTapeRobotics-2.3") = 2.2 with
   ARC_MAX_TURN_DEG 15 -> 25. The user said the snap only worked when looking almost straight at the
   tags; at a pose about 20 deg off, the Limelight still saw 3-4 tags with steady positions, so the 15 deg
   start limit was the bottleneck. Compiles; deployed to the Control Hub 2026-10-29 18:02 (app restarted); NOT yet tested. Camera view is about +/-27 deg,
   so do not raise past 25.
   2026-10-29 ~18:10: Snap Teach showed tag height 1330 (expect 1268). Live data: 1268 at a tilt of
   40.3 deg, so SnapAim.CAM_PITCH_DEG changed 44.0 -> 40.3 (shared by 2.1, 2.2, 2.3 and SnapTeach).
   Deployed 2026-10-29 18:07 (app restarted). Snap Teach must be redone (old teach used 44.0).
   Re-taught 2026-10-29 18:09 with tilt 40.3: snap_teach.txt = "1125.52,0.015" (old 1073.56,-2.416).
   18:15 Limelight sample at the primary: 2-4 tags per frame (32 and 33 drop out most), 26 fps,
   exposure now 2247 (user raised it), flicker 60 Hz, downscale 1, quality threshold 2. Suggested:
   exposure back to about 1100-1300 (less blur, less glare from the light above the cell), then
   quality threshold 1.5 if still short of tags.
   DONE 2026-10-30: `DuckTapeRobotics_2_4.java` (TeleOp "DuckTapeRobotics-2.4") = 2.3 without the
   pre-match confirm (no more "press gamepad1 X or the robot stays disabled"). Alliance still defaults
   to RED and gamepad2 Start still swaps it. Compiles; NOT yet deployed.
   DONE 2026-10-30: `DuckTapeRobotics_2_5.java` (TeleOp "DuckTapeRobotics-2.5") = 2.4 plus servo5,
   config name "FJF2", which copies servo4 (gamepad1 Y toggle, same UP/DOWN angles and D-pad fine-tune).
   SERVO5_REVERSED = true (same as servo4); flip it if FJF2 is mounted mirrored. Uses tryGet, so the
   robot still runs without FJF2 in the config. FJF2 must be added to robot config DuckTape1. Compiles;
   The user built and deployed it from Android Studio on 2026-10-30 (adb showed the hub offline, so not
   confirmed from here).
   DONE 2026-10-30: `DuckTapeRobotics_2_6.java` (TeleOp "DuckTapeRobotics-2.6") = 2.5 plus color sensor
   "color_NC" in the nectar chamber: servo3 (gamepad2 X, nectar chamber release) only starts when
   color_NC reads nothing closer than NC_EMPTY_DISTANCE_CM (4.0 cm, untuned). Missing sensor = no check.
   Not a color-sort sensor. Free I2C buses: Control Hub 3, Expansion Hub 2 and 3. Compiles; NOT deployed.
   DONE 2026-10-30: `DuckTapeRobotics_2_7.java` (TeleOp "DuckTapeRobotics-2.7") = 2.6 with
   NC_EMPTY_DISTANCE_CM 4.0 -> 2.0 (full only when under 2 cm). Compiles; NOT deployed.
   DONE 2026-10-30: `DuckTapeRobotics_2_8.java` (TeleOp "DuckTapeRobotics-2.8") = 2.7 with
   SERVO5_REVERSED true -> false (FJF2 turned the wrong way on the robot). Compiles; NOT deployed.
   2026-10-30: per the user's explicit request, 2.8 was edited IN PLACE (no 2.9) to add an FJF2 start
   offset (gamepad2 triggers, saved to a file). Then, also in place, REPLACED by a fixed FJF2 start:
   SERVO5_HOME_DEG = 8, not adjustable; DOWN still copies servo4. An old ducktape_fjf2.txt on the hub is unused.
   Then (user: "add the adjustment back") the adjust returned ON TOP of the 8: gamepad2 RT +1 / LT -1 degree,
   saved as an absolute angle in ducktape_fjf2_start.txt (8 used only when no file). Telemetry "FJF2 start".
   The servo4 START adjust briefly added was REMOVED (an unused ducktape_servo4_start.txt may exist on the hub).
   Replaced by END (DOWN) adjusts, in place: gamepad2 left stick up/down = servo4 end +/-1, right stick
   up/down = FJF2 end +/-1 (FJF2 now has its OWN end, default 87, no longer copies servo4). Saved in
   ducktape_servo4_end.txt and ducktape_fjf2_end.txt. Gamepad1 D-pad while DOWN moves both ends together.
   Telemetry "Servo4 end" / "FJF2 end".
   Compiles; NOT deployed.
   NEXT: deploy 2.8, line FJF2 up with the triggers, check NC_EMPTY_DISTANCE_CM against the "Nectar
   chamber" telemetry line, check FJF2 direction unhooked, Pinpoint push test, first snap. ARC_MAX_POWER is 0.40, not 0.25; from
   75 mm off the drive asks for only 0.003 x 75 = 0.23 power. Watch for "time limit" on big turns
   (ARC_TIME_LIMIT_SEC 1.5).
3. Fix `newauto.java`: remove the 10 s `sleep` from INIT, and fix the comments that no longer match
   the code.
4. Decide on one set of POLLEN numbers and copy it to every op mode: P 160 or 180, RPM, and fire
   window.
5. Fix the motor names in `MecanumTeleOp`, or archive it. Archive `ShooterLEDControl` and the unused
   `TelemetryLog` if they are not wanted.
6. Style cleanup to the house rules (`SnapAim`, `SnapTeach`, the TeleOp).

7. Field planner (2026-10-30): https://claude.ai/artifact/Vwst8LfWSa5DXEuPSBkjwm (private). Blank 6x6 tile field in
   Pedro inches (x from red wall, y from audience wall), waypoints + pen + manual-image background, copies Java
   Poses/PathChain. Cell/goal positions are NOT drawn (no full BIOBUZZ layout in the project).

8. The "RED Hive + 2 Flowers V00" autonomous was removed at the user's request (2026-10-30): moved to
   Archive\2026-10-30\src\main\java\org\firstinspires\ftc\teamcode\DuckTapeAutoRedHiveFlowersV00.java (not compiled there).

9. NEW (2026-10-30, from the user's full spec): DuckTapeAutoTwoFlowersBaseV00.java (shared state machine, OpMode/iterative)
   + DuckTapeAutoTwoFlowersRedV00 ("RED Two Flowers V00") + DuckTapeAutoTwoFlowersBlueV00 ("BLUE Two Flowers V00").
   Real poses from the user. Intake steps end at 4 balls on limit1/limit2 (tryGet) or 1.5 s. FIRE_ON_TIMEOUT = false
   (a shot that never settles in 2.5 s is skipped). RPM gate uses a 6-reading average. Compiles; NOT run on the robot.

10. V01 of the two-flower auto ("RED/BLUE Two Flowers V01", Base/Red/Blue V01 files): estimate for V00 is about
   33-36 s (best case ~26), so V01 adds a 30 s match clock: skip flower 2 if < 14 s left after step 8, skip last
   volley if < 9 s left after step 11, and go park from anywhere at < 4 s left. Telemetry: clock, per-step times,
   park reason. V00 kept unchanged. Compiles; NOT run on the robot. Time it on the field and tune.

## 6. Gotchas
- Deploys made with Android Studio "Apply Changes" sometimes never reached the hub. Use Clean Project
  and then the green Run button.
- Deploy without Android Studio, from the repo root (`C:\FTC\FTCWorkingTagsPedro`):
  `ANDROID_SERIAL=192.168.43.1:5555 ./gradlew :TeamCode:installDebug`. adb lists TWO devices (the Control
  Hub at 192.168.43.1:5555 and a USB phone ZY326GWTVZ). Without ANDROID_SERIAL, installDebug installs to
  both. adb is at `%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe`.
- Android Studio's run-target dropdown was set to the USB phone early on 2026-10-29; the user then
  switched it to "REV Robotics Control Hub v1.0". Check it before pressing Run.
- Claude Code CLI 2.1.312 was installed 2026-10-29 at `C:\Users\BoyBlue\.local\bin\claude.exe`, but that folder
  must be on the user PATH (and Android Studio restarted) before `claude` works in its terminal.
  Later on 2026-10-29 `where claude` found it, so the PATH is fixed.
- Sloth fast deploy is NOT active: build.gradle adds the Load classpath but never applies the plugin, so
  no deploySloth task exists. Every deploy is a full install.
- The Pedro tuning op mode starts at pose (72, 72), not (0, 0).
- **Two places in the V05 report are wrong:**
  - It says "hold the right trigger"; the built code uses the right stick button.
  - It calls the aim point "the middle of the cell floor"; it is really 75 mm behind the lip, as
    `SnapAim.java` says.
- `Claude.Usability.txt` holds personal notes recommending bypass-permissions mode. They are notes,
  not project instructions.

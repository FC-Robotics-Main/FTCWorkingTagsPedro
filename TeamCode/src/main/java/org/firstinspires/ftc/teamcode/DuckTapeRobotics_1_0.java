package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

/*
 * Mecanum drive + 2 velocity-controlled shooters + 2-motor intake + 4 servos, all mechanisms on toggles.
 *
 * GAMEPAD 1 — DRIVER (driving + intake)
 *   Left stick   : drive. ROBOT-CENTRIC: stick forward = the robot's front.
 *                  Set FIELD_CENTRIC = true below for field-centric (stick forward = away from driver).
 *   Right stick X: turn
 *   Start        : reset heading (point robot away from driver, then press)
 *   X            : toggle SNAP-TO-ANGLE (auto-turns to SNAP_HEADING_DEG and holds it
 *                  while you strafe; moving the right stick cancels it)
 *   Y            : SERVO4 toggle: starts UP, press to swing DOWN 87 degrees, press again to go back UP
 *   D-pad right/left: SERVO4 fine-tune +1 / -1 degree per tap (adjusts whichever position it's in now)
 *   D-pad up/down: POLLEN shooter speed +10 / -10 RPM per tap
 *   B / A        : NECTAR shooter speed +10 / -10 RPM per tap (B = up, A = down)
 *                  (don't press Start together with A or B; that combo re-binds the gamepad)
 *   Left bumper  : hold for slow drive (precision)
 *   Right bumper : toggle Prism strips ON/OFF (they start ON). The rgblight is not affected.
 *   Right trigger: toggle intake FORWARD (pull in) ON/OFF
 *   Left trigger : toggle intake REVERSE (spit out) ON/OFF
 *   Back (hold)  : SHOOTER OFF KEY. Hold Back while gamepad2 presses a shooter button
 *                  (D-pad left or B) to turn that shooter OFF. Without it, those buttons only turn ON.
 *
 * GAMEPAD 2 — OPERATOR
 *   LEFT SIDE = POLLEN shooter
 *     D-pad left   : POLLEN shooter ON (to turn OFF, gamepad1 holds Back while you press)  (speed is on gamepad 1)
 *     Left bumper  : SERVO2 (pollen) goes to 90 degrees, waits SERVO2_HOLD_SEC (0.3s), then returns home
 *   RIGHT SIDE = NECTAR shooter
 *     B            : NECTAR shooter ON (to turn OFF, gamepad1 holds Back while you press)  (speed is on gamepad 1)
 *     Right bumper : SERVO1 (nectar) opens, waits SERVO1_HOLD_SEC (0.45s), then returns home
 *     Back         : NECTAR OVERRIDE (toggle): press once = SERVO1 opens and STAYS open, skipping the
 *                  fire gate (any RPM, even with the shooter off). Press Back again = SERVO1 closes.
 *                  Right bumper does nothing while it's held open. Shot log marks it "NECTAR OVR".
 *     X            : SERVO3 (nectar chamber) goes up 45 degrees, waits SERVO3_HOLD_SEC (0.45s), then returns home
 *   FIRE GATE: pressing the button for SERVO1/2 only starts its open-wait-return cycle if THEIR
 *     shooter is ON and its actual RPM has been within FIRE_TOLERANCE_RPM (45) of target for
 *     the fire settle time (starts at 0.30s). If the RPM isn't right, the press is ignored (short rumble on gamepad 2,
 *     no telemetry message). Press again once it's settled.
 *     SERVO3 is NOT gated; it always runs when pressed.
 *     Y            : FIRE WINDOW step: each press +10 RPM. Anything past 80 wraps back to 10.
 *                  Starts at +/-45 every match: 45 -> 55 -> 65 -> 75 -> 10 -> 20 -> ... -> 80 -> 10 ...
 *                  Shown on telemetry "Fire window".
 *     Right stick button (press the stick down): fire settle time +0.1s per tap
 *     Left stick button  (press the stick down): fire settle time -0.1s per tap
 *                  (limits 0-2.0s; 0 = fire the moment it's in band; current value is on telemetry "Fire settle")
 *   SHOT LOG (still recorded, NOT on telemetry right now): the last 3 shots (servo1 = NECTAR, servo2 = POLLEN), newest first.
 *     For each: RPM when fired / target, how long until it was back within the fire tolerance,
 *     the lowest RPM after the shot and the highest RPM (overshoot) during the recovery.
 *   Everything else
 *     D-pad right  : SHAKE: robot strafes left/right quickly for SHAKE_TIME_SEC to unstick balls
 *     D-pad up     : manually steps the LIMIT SWITCH COUNT
 *                  (and first Prism strip color): 0 OFF -> 1 RED -> 2 GREEN -> 3 BLUE -> 4 WHITE -> back to 0 OFF
 *                  (from 4 or higher, one press resets to 0)
 *     Start        : second Prism strip: swaps RED <-> BLUE (starts RED)
 *                  This is also OUR BALL COLOR for color sorting (see below).
 *                  (don't press Start together with A or B; that combo re-binds the gamepad)
 *     D-pad down   : ALERT: second strip runs the siren AND the
 *                  intake runs in REVERSE for ALERT_SEC (3s), then both go back to how they were
 *   Rumble/LED   : flywheel "ready to fire" feedback (gamepad only)
 *   Prism strip 3: NECTAR shooter target RPM.  Prism strip 4: POLLEN shooter target RPM.
 *                  (set which shooter is which with NECTAR_IS_LEFT_SHOOTER)
 *                  <2200 SIREN, 2200 RED, 2300 ORANGE, 2400 GREEN, 2500 BLUE, 2600 WHITE,
 *                  >2600 SPECIAL (fast full rainbow)
 *
 * COLOR SORTING (automatic)
 *   Light 2 RED  + sensor sees RED ball  -> nothing
 *   Light 2 RED  + sensor sees BLUE ball -> ALERT (siren on light 2 + intake reverse for ALERT_SEC)
 *   Light 2 BLUE + sensor sees BLUE ball -> nothing
 *   Light 2 BLUE + sensor sees RED ball  -> ALERT
 *   If the wrong ball is still in front of the sensor when the alert ends, it fires again.
 *   Set COLOR_SORT_ON = false to turn it off completely.
 *
 * LIMIT SWITCH (automatic) -> drives the FIRST Prism strip color
 *   TWO switches (limit1 + limit2) share ONE count. A press on EITHER switch adds 1
 *     (limit1 then limit2 = 2). Each switch only counts once per press, even if held.
 *   Count starts at LIMIT_START_COUNT (0 = no balls):
 *     0 = OFF (strip dark, no balls), 1 = RED, 2 = GREEN, 3 = BLUE, 4 = WHITE.
 *     It keeps counting past 4 (5, 6, 7...); the strip stays WHITE for 4 and up.
 *   Gamepad2 D-pad up steps the count by hand (from 4 or higher it goes back to 0, so it's also the reset).
 *   Holding the switch down only counts once. Counts only while the OpMode is running (not during INIT).
 *
 * HARDWARE CONFIG NAMES (must match the Driver Station config exactly)
 *   FLM (front left motor), FRM (front right motor),
 *   BLM (back left motor),  BRM (back right motor),
 *   SLM (shooter left motor), SRM (shooter right motor),
 *   ILM (intake left motor),  IRM (intake right motor)
 *   servo_NT (servo1 in the code), servo_PT (servo2 in the code), servo_NC (servo3 in the code), servo_FJ (servo4 in the code) (servo ports, configured as "Servo")
 *   imu (the Control Hub's built-in IMU, I2C bus 0; usually already in the config)
 *   prism (goBILDA Prism RGB LED Driver, Control Hub I2C bus 1)
 *   color1 (REV Color Sensor V3, CONTROL HUB I2C bus 2)
 *   color2 (REV Color Sensor V3, EXPANSION HUB I2C bus 0)
 *     These are I2C sensors - they do NOT work in the digital ports.
 *     Two REV V3 sensors can NOT share a bus (same I2C address) - one sensor per bus.
 *   limit1 (goBILDA micro limit switch, Control Hub DIGITAL port 0, configured as "Digital Device")
 *   limit2 (goBILDA micro limit switch, Control Hub DIGITAL port 2, configured as "Digital Device")
 *     Each switch on its own 4-pin digital connector (limit1 on the "0-1" connector, limit2 on "2-3").
 *     Wiring to the 4-pin digital port (GND, 3.3V, n+1, n):
 *       COM -> pin 4 (channel n)    NO -> GND    NC -> 3.3V
 *     Released = HIGH, pressed = LOW.
 *   rgblight (goBILDA RGB Indicator Light, servo port, configured as "Servo")
 *     INTAKE INDICATOR: GREEN = intake forward, RED = intake reverse (incl. ALERT), OFF = stopped.
 *
 * PRISM LIGHTS NEED the goBILDA driver files in TeamCode/.../teamcode/Prism/
 *   (Color.java, Direction.java, GoBildaPrismDriver.java, PrismAnimations.java)
 *
 * SHOOTER MOTORS MUST HAVE ENCODER CABLES PLUGGED IN.
 * Without them, velocity mode will run the motor at full power or not at all.
 */
@TeleOp(name = "DuckTapeRobotics-1.0", group = "TeleOp")
public class DuckTapeRobotics_1_0 extends LinearOpMode {

    // ---- Shooter motor setup ----
    // Encoder ticks per revolution of the motor OUTPUT shaft. Common values:
    //   goBILDA 5203 6000 RPM (1:1)    : 28
    //   goBILDA 5203 1620 RPM (3.7:1)  : 103.8
    //   REV HD Hex bare (no gearbox)   : 28
    //   REV Core Hex                   : 288
    private static final double SHOOTER_TICKS_PER_REV = 28;
    private static final double SHOOTER_MAX_RPM       = 6000;  // free speed of the motor

    // ---- Shooter PIDF ----
    // Tune with the "Shooter PIDF Tuner V2" OpMode, then paste its "PASTE INTO TELEOP" lines here.
    // false = use the hub's built-in values (no longer shown on telemetry).
    // Set true after tuning. The hub forgets PIDF at power-off, so this code sets it every start.
    private static final boolean USE_CUSTOM_PIDF = true;   // tuned 2026-10-21 at 2400 RPM, 12.8-12.9 V
    // LEFT shooter motor (SLM) — NECTAR while NECTAR_IS_LEFT_SHOOTER = true
    private static final double SHOOTER_P_LEFT  = 180.0;   // raised from 120 on 2026-10-22 to match POLLEN
    private static final double SHOOTER_I_LEFT  = 0.0;
    private static final double SHOOTER_D_LEFT  = 0.0;
    private static final double SHOOTER_F_LEFT  = 13.0;
    // RIGHT shooter motor (SRM) — POLLEN while NECTAR_IS_LEFT_SHOOTER = true
    private static final double SHOOTER_P_RIGHT = 160.0;
    private static final double SHOOTER_I_RIGHT = 0.0;
    private static final double SHOOTER_D_RIGHT = 0.0;
    private static final double SHOOTER_F_RIGHT = 13.7;   // re-fit 2026-10-22 for the 2200 RPM match speed (14.8 was for 2400)

    // ---- Voltage-scaled F (optional, advanced; leave false until the basic tune works) ----
    // Turn on only if recovery gets much worse late in a match. F is scaled by
    // PIDF_TUNED_VOLTS / battery volts, so a sagging battery still gets the right base power.
    // Only active when USE_CUSTOM_PIDF = true.
    private static final boolean VOLTAGE_COMP_F   = false;
    private static final double  PIDF_TUNED_VOLTS = 12.9;   // battery voltage during tuning (the tuner prints it)
    private static final double  VCOMP_UPDATE_SEC = 0.5;    // how often to check the battery
    private static final double  VCOMP_MIN_CHANGE = 0.02;   // only resend F if the scale moved 2% or more
    private static final double  VCOMP_MIN_SCALE  = 0.85;   // safety limits on the scale
    private static final double  VCOMP_MAX_SCALE  = 1.25;

    // Shooter speed. Each shooter has its own target.
    //   POLLEN shooter: gamepad1 D-pad up = +RPM_STEP, D-pad down = -RPM_STEP
    //   NECTAR shooter: gamepad1 B        = +RPM_STEP, A          = -RPM_STEP
    private static final double START_RPM_LEFT  = 2410;  // NECTAR (left shooter) speed at the start of the match
    private static final double START_RPM_RIGHT = 2400;  // POLLEN (right shooter) speed at the start of the match
    private static final double RPM_STEP        = 10;    // change per tap (was 100)
    private static final double MIN_RPM         = 1000;  // D-pad down stops here

    // ---- Fire gate (servo1 = nectar, servo2 = pollen; servo3 is NOT gated) ----
    // A fire servo will only move if its shooter is ON and the ACTUAL RPM has stayed within
    // FIRE_TOLERANCE_RPM of target for at least FIRE_SETTLE_SEC (so a flywheel just passing
    // through the target while overshooting doesn't count). Set FIRE_SETTLE_SEC = 0 for "in band right now".
    // Only the START of a fire is gated: once the servo opens it finishes its timed cycle even though
    // the RPM dips as the ball goes through.
    private static final boolean FIRE_GATE_ON       = true;
    // The hub reports speed in steps of about 43 RPM (20 ticks/s). A +/-45 band is 90 RPM wide,
    // so it always holds at least one reading (two at 2410 and 2250). If you change a target and
    // shots start getting blocked on every flicker, raise this back toward 60.
    private static final double  FIRE_TOLERANCE_RPM = 45;   // STARTING window; gamepad2 Y steps it. Below ~22 it can block every shot
    private static final double  FIRE_TOL_STEP      = 10;   // +RPM per Y press
    private static final double  FIRE_TOL_MIN       = 10;   // where it wraps back to
    private static final double  FIRE_TOL_MAX       = 80;   // highest it goes
    private static final double  FIRE_SETTLE_SEC    = 0.30; // STARTING settle time; gamepad2 stick clicks change it during the match
    private static final double  FIRE_SETTLE_STEP   = 0.1;  // change per stick click
    private static final double  FIRE_SETTLE_MIN    = 0.0;
    private static final double  FIRE_SETTLE_MAX    = 2.0;
    // ---- Shot log (telemetry) ----
    // Each servo1 (nectar) / servo2 (pollen) fire starts a record. For SHOT_LOG_SEC after the fire it
    // tracks that shooter's lowest RPM, highest RPM, and the time until it's back within FIRE_TOLERANCE_RPM.
    // Firing the same shooter again before SHOT_LOG_SEC is up ends the previous record early.
    private static final int    SHOT_LOG_SIZE = 3;     // how many shots to show
    private static final double SHOT_LOG_SEC  = 4.0;   // how long each shot is watched (POLLEN takes ~2 s to recover)

    // Rumble/LED "ready" feedback uses the same tolerance, so the buzz means "you can fire now".

    // ---- Other tunables ----
    private static final double INTAKE_POWER = 1.0;
    private static final double SLOW_MODE    = 0.4;

    // ---- Servo1 (Right bumper) ----
    // setPosition() takes 0.0-1.0, which covers the servo's FULL travel.
    // SERVO_RANGE_DEG must match your servo or "90 degrees" will be wrong:
    //   Most standard hobby servos       : 180
    //   REV Smart Servo (default mode)   : 270
    //   goBILDA 2000-series (dual mode)  : 300
    // If it moves the wrong way, set SERVO1_REVERSED = true.
    private static final double  SERVO_RANGE_DEG   = 180;
    private static final double  SERVO1_HOME_DEG   = 15;   // where it rests at start
    private static final double  SERVO1_TARGET_DEG = 90;  // where the button sends it (raise/lower to adjust how far it goes)
    private static final boolean SERVO1_REVERSED   = true;
    private static final double  SERVO1_HOLD_SEC   = 0.45;  // how long it stays open before returning

    // ---- Servo2 (gamepad2 Left bumper) ----
    // Same idea as servo1, with its own settings. Uses the same SERVO_RANGE_DEG,
    // so if servo2 is a different model, give it its own range.
    private static final double  SERVO2_HOME_DEG   = 30;   // kept off 0 so the servo isn't jammed at its end stop
    private static final double  SERVO2_TARGET_DEG = 110;  // 80-degree swing from home (set 120 for a 90-degree swing)
    private static final boolean SERVO2_REVERSED   = false; // set true if it moves the wrong way
    private static final double  SERVO2_HOLD_SEC   = 0.3;   // how long it stays out before returning

    // ---- Servo3 (gamepad2 A) ----
    // Goes UP (home + 45 degrees) when A is pressed, returns home after SERVO3_HOLD_SEC.
    // Uses the same SERVO_RANGE_DEG as the others.
    // If it goes DOWN instead of up, set SERVO3_REVERSED = true.
    // If it hits its end stop at home, raise SERVO3_HOME_DEG (e.g. 10); the 45-degree swing stays the same.
    private static final double  SERVO3_HOME_DEG   = 0;
    private static final double  SERVO3_SWING_DEG  = 45;   // how far UP goes from home
    private static final double  SERVO3_UP_DEG     = SERVO3_HOME_DEG + SERVO3_SWING_DEG;
    private static final boolean SERVO3_REVERSED   = false;
    private static final double  SERVO3_HOLD_SEC   = 0.45;  // how long it stays up before returning

    // ---- Servo4 (gamepad1 Y) ----
    // TOGGLE: starts UP at SERVO4_HOME_DEG. Press Y to swing DOWN 87 degrees, press Y again to go back up.
    // No timer; it stays where you put it. Uses the same SERVO_RANGE_DEG as the others.
    // If it buzzes at the bottom (end stop), lower SERVO4_SWING_DEG (e.g. 70).
    private static final double  SERVO4_HOME_DEG   = 0;    // start position (up) - at the end of travel; raise to 5-10 if it buzzes at rest
    private static final double  SERVO4_SWING_DEG  = 87;   // how far DOWN it goes when toggled (0 + 87 = 87), fixed
    private static final double  SERVO4_OPEN_DEG   = SERVO4_HOME_DEG + SERVO4_SWING_DEG;  // 87 deg = down
    private static final boolean SERVO4_REVERSED   = true;   // reversed: down swing now goes the other way
    // Fine-tune during the match: gamepad1 D-pad right/left moves servo4 by this many degrees per tap.
    // It adjusts whichever position servo4 is in right now (UP or DOWN), and remembers it.
    private static final double  SERVO4_STEP_DEG   = 1;

    // ---- Shake (gamepad2 Y) ----
    // One press = robot strafes left, right, left, right... to knock stuck balls loose.
    // Drive sticks are ignored while it shakes. Press again after it finishes to repeat.
    private static final double SHAKE_POWER    = 0.5;   // how hard it shakes (0-1); raise if balls stay stuck
    private static final double SHAKE_FLIP_SEC = 0.08;  // time each direction; lower = faster shake
    private static final double SHAKE_TIME_SEC = 0.64;  // total shake time (8 flips, ends where it started)

    // ---- Stick feel ----
    private static final double STICK_DEADZONE = 0.05;  // ignore tiny stick drift
    private static final double STICK_CURVE    = 2.0;   // 1 = linear, 2 = squared (finer low-speed control)

    // ---- Drive mode ----
    // false = ROBOT-CENTRIC (stick forward = robot's front). Use this while testing.
    // true  = FIELD-CENTRIC (stick forward = away from driver). Needs HUB_LOGO/HUB_USB set correctly.
    private static final boolean FIELD_CENTRIC = false;

    // ---- Heading hold (fixes curving) ----
    // When you drive without touching the turn stick, the IMU keeps the robot
    // pointed the same way, correcting for a weak/strong wheel or uneven drag.
    private static final boolean HEADING_HOLD   = false;
    private static final double  HOLD_KP        = 1.5;   // raise if it still curves, lower if it wiggles
    private static final double  HOLD_MAX_POWER = 0.3;   // cap so correction never overpowers the driver

    // ---- Drive balance (set here in code; no buttons) ----
    // 0.0 = even. Robot pulls RIGHT -> set positive (e.g. 0.05) to slow the LEFT side.
    //             Robot pulls LEFT  -> set negative (e.g. -0.05) to slow the RIGHT side.
    private static final double DRIVE_BALANCE = 0.0;

    // ---- Per-wheel trim ----
    // Multiplies each wheel's power. 1.0 = normal. If one motor is faster
    // (e.g. a different gearbox), lower its trim, e.g. TRIM_BL = 0.85.
    private static final double TRIM_FL = 1.0;
    private static final double TRIM_FR = 1.0;
    private static final double TRIM_BL = 1.0;
    private static final double TRIM_BR = 1.0;

    // ---- Snap-to-angle (press X) ----
    // Heading the robot turns to, in degrees. 0 = facing away from the driver
    // (the direction it faced at init / last Start press). Positive = turned LEFT.
    // Set this to whatever angle points your shooter at the goal.
    private static final double SNAP_HEADING_DEG   = 0;
    private static final double SNAP_KP            = 1.2;   // turn strength; raise if sluggish, lower if it wobbles
    private static final double SNAP_MAX_POWER     = 0.7;   // fastest it will spin while snapping
    private static final double SNAP_MIN_POWER     = 0.06;  // overcomes friction near the target
    private static final double SNAP_TOLERANCE_DEG = 1.5;   // "close enough" window

    // ---- Color sorting ----
    // Light 2's color (gamepad2 Start) is OUR color. The OTHER color triggers the ALERT
    // (siren on light 2 + intake reverse for ALERT_SEC).
    // Tune with the telemetry lines "Sensor color1 ...": hold a red ball, then a blue ball,
    // in front of the sensor and read the hue/sat/dist numbers.
    private static final boolean  COLOR_SORT_ON       = true;
    private static final String[] COLOR_SENSOR_NAMES  = {"color1", "color2"};  // either one seeing the wrong color triggers the alert
    private static final float    COLOR_GAIN          = 2.0f;   // raise if readings are dim / sat is low
    private static final double   BALL_DISTANCE_CM    = 4.0;    // ball counts as "there" if closer than this
    private static final float    MIN_SATURATION      = 0.35f;  // below this = gray/white/empty, ignored
    // Hue is 0-360. Red wraps around 0, so red = hue below RED_HUE_LOW OR above RED_HUE_HIGH.
    private static final float    RED_HUE_LOW         = 30;
    private static final float    RED_HUE_HIGH        = 330;
    private static final float    BLUE_HUE_MIN        = 190;
    private static final float    BLUE_HUE_MAX        = 260;

    // ---- Limit switches (goBILDA micro limit switches, digital ports "limit1" + "limit2") ----
    // Both switches add to ONE shared count.
    // Wiring: COM -> signal (pin 4), NO -> GND, NC -> 3.3V. Pressed = LOW.
    // If the count goes up when you RELEASE instead of press, set LIMIT_PRESSED_IS_LOW = false.
    // If telemetry says PRESSED all the time, COM is on the wrong pin (move it to pin 4).
    private static final String[] LIMIT_SWITCH_NAMES  = {"limit1", "limit2"};
    private static final boolean LIMIT_PRESSED_IS_LOW = true;
    private static final double  LIMIT_DEBOUNCE_SEC   = 0.02;  // ignore contact bounce for 20 ms; lower to 0.01 if fast hits get missed
    // The count IS the first strip's color: 0 = OFF (no balls), 1-4 = STRIP1_COLORS (RED, GREEN, BLUE, WHITE).
    private static final int     LIMIT_START_COUNT    = 0;     // count at the start of the match (0-4); set 3 if you preload 3

    // ---- Prism LED lights (gamepad1 Right bumper) ----
    // The 4 strips are daisy-chained, so the Prism sees ONE long string of LEDs.
    // List each strip's LED count IN THE ORDER THEY ARE CHAINED, starting from the
    // strip plugged into the Prism. Kit strips: short = 6 LEDs, long = 12 LEDs.
    // If the wrong strip gets the wrong pattern, reorder these numbers.
    private static final int[] STRIP_LEDS      = {12, 12, 6, 6};
    private static final int   LED_BRIGHTNESS  = 60;     // 0-100
    private static final boolean LIGHTS_START_ON = true;

    // First strip in the chain = LIMIT SWITCH COUNT. Count 0 = strip dark (no balls),
    // count 1 = first color here, count 4 AND ABOVE = last color here.
    // Limit switch presses and gamepad2 D-pad up both step through these.
    private static final Color[]  STRIP1_COLORS      = {Color.RED, Color.GREEN, Color.BLUE, Color.WHITE};
    private static final String[] STRIP1_COLOR_NAMES = {"RED", "GREEN", "BLUE", "WHITE"};

    // Second strip: gamepad2 Start swaps red/blue. Gamepad2 D-pad down = siren + intake reverse.
    private static final double ALERT_SEC = 3.0;   // how long the siren + reverse intake lasts

    // Third strip = NECTAR shooter's target RPM. Fourth strip = POLLEN shooter's target RPM.
    // true  = nectar is the LEFT shooter motor (SLM), pollen is the RIGHT shooter motor (SRM).
    // false = swapped. The gamepad layout follows nectar/pollen, not left/right motor:
    // RPM +/- is on gamepad 1 (pollen = D-pad up/down, nectar = B/A); ON/OFF stays on gamepad 2.
    private static final boolean NECTAR_IS_LEFT_SHOOTER = true;
    //   below 2200 = SIREN, 2200 = RED, 2300 = ORANGE, 2400 = GREEN, 2500 = BLUE, 2600 = WHITE,
    //   above 2600 = SPECIAL: fast full rainbow
    private static final double[] RPM_LIGHT_STEPS  = {2200, 2300, 2400, 2500, 2600};
    private static final Color[]  RPM_LIGHT_COLORS = {Color.RED, Color.ORANGE, Color.GREEN, Color.BLUE, Color.WHITE};
    private static final String[] RPM_LIGHT_NAMES  = {"RED", "ORANGE", "GREEN", "BLUE", "WHITE"};

    // ---- goBILDA RGB Indicator Light (servo port) ----
    // INTAKE INDICATOR: GREEN = intake forward, RED = intake reverse (LT or ALERT), OFF = stopped.
    // 0.500 is goBILDA's green value and 0.0 is off, for the FTC DEFAULT servo
    // pulse range (600-2400us), so do NOT call setPwmRange() on this light.
    // RED: goBILDA lists ~0.279, but that's right at the edge where the light switches off
    // (0.277 = ~1099us, the light needs ~1100us+), so some lights stay dark. 0.29 sits safely inside red.
    // If it looks orange, lower toward 0.283; if it's still dark, raise toward 0.30.
    // Do NOT reverse this servo or the color comes out wrong.
    private static final double RGB_GREEN = 0.500;
    private static final double RGB_RED   = 0.29;
    private static final double RGB_OFF   = 0.0;

    // ---- Control Hub mounting (MUST match how the hub sits on YOUR robot) ----
    // LOGO = which way the REV logo on the hub faces.
    // USB  = which way the USB ports face.
    // Options: UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;   // USB ports face the robot's LEFT side

    // ---- Hardware ----
    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeLeft, intakeRight;
    private DcMotorEx shooterLeft, shooterRight;
    private Servo servo1, servo2, servo3, servo4;
    private ServoImplEx rgbLight;
    private IMU imu;
    private List<LynxModule> hubs;
    private GoBildaPrismDriver prism;
    private NormalizedColorSensor[] colorSensors;
    private DigitalChannel[] limitSwitches;

    // One animation per strip
    private final PrismAnimations.Solid         stripSolid  = new PrismAnimations.Solid(Color.RED);
    private final PrismAnimations.Solid         strip2Solid = new PrismAnimations.Solid(Color.RED);
    private final PrismAnimations.PoliceLights  stripSiren  = new PrismAnimations.PoliceLights();
    // Strips 3 and 4 each show one shooter's target RPM (see RpmStrip below).
    private final RpmStrip nectarStrip = new RpmStrip(LayerHeight.LAYER_2, NECTAR_IS_LEFT_SHOOTER,  STRIP_LEDS[2]);
    private final RpmStrip pollenStrip = new RpmStrip(LayerHeight.LAYER_3, !NECTAR_IS_LEFT_SHOOTER, STRIP_LEDS[3]);
    private boolean lightsOn = false;
    private boolean lastRB1;
    private int limitCount = LIMIT_START_COUNT;   // 0 = no balls (strip 1 dark), 1-4 = RED/GREEN/BLUE/WHITE
    private boolean lastRT1, lastLT1;               // gamepad1 triggers (intake)
    private boolean lastRSB2, lastStart2, lastLSB2, lastDpadUp2, lastDpadDown2;
    private double  fireToleranceRpm = FIRE_TOLERANCE_RPM;   // live fire window (gamepad2 Y steps it)
    private double  fireSettleSec    = FIRE_SETTLE_SEC;      // live settle time (gamepad2 stick clicks +/- FIRE_SETTLE_STEP)
    private boolean strip2Blue = false;     // false = RED, true = BLUE  (also = our ball color)
    private boolean alertActive = false;    // siren + reverse intake running
    private final ElapsedTime alertTimer = new ElapsedTime();
    private static final double TRIGGER_PRESSED = 0.5;  // trigger counts as "pressed" past halfway

    // ---- Color sensor state ----
    private enum BallColor { NONE, RED, BLUE }
    private BallColor[] sensorColor;   // what each sensor sees this loop
    private float[]     sensorHue, sensorSat;
    private double[]    sensorDist;
    private String      lastSortEvent = "none yet";

    // ---- Limit switch state ----
    // (the count itself is limitCount, which also drives strip 1, so the light and the count can never disagree)
    // One entry per switch: debounced state + its own debounce timer.
    private boolean[]     limitWasPressed;
    private ElapsedTime[] limitDebounce;

    // ---- Shooter PIDF state ----
    private String pidfAsFoundLeft = "", pidfAsFoundRight = "";   // what the hub had before this OpMode touched it
    private String pidfActiveLeft  = "", pidfActiveRight  = "";   // what is running now
    private double appliedFScale = 1.0;                            // voltage-comp F multiplier currently on the hub
    private final ElapsedTime vcompTimer = new ElapsedTime();

    // ---- State ----
    private double targetRpmLeft  = START_RPM_LEFT;   // current left shooter target
    private double targetRpmRight = START_RPM_RIGHT;  // current right shooter target
    private boolean intakeOn = false;
    private boolean intakeReverse = false;
    private double rgbLightPos = RGB_OFF;   // intake indicator light's current color value
    private boolean shooterLeftOn  = false;
    private boolean shooterRightOn = false;
    private boolean servo1Out = false;  // false = home, true = at SERVO1_TARGET_DEG
    private final ElapsedTime servo1Timer = new ElapsedTime();  // time since servo1 was sent out
    private boolean servo3Up = false;   // false = home, true = up 45 degrees
    private final ElapsedTime servo3Timer = new ElapsedTime();
    private boolean servo2Out = false;
    private final ElapsedTime servo2Timer = new ElapsedTime();
    private boolean servo4Open = false; // false = up (home), true = down
    private double  servo4HomeDeg = SERVO4_HOME_DEG;  // current UP angle (D-pad adjusts it)
    private double  servo4OpenDeg = SERVO4_OPEN_DEG;  // current DOWN angle (D-pad adjusts it)
    private boolean lastDpadUp1, lastDpadDown1;
    private boolean shaking = false;    // shake in progress
    private final ElapsedTime shakeTimer = new ElapsedTime();
    private boolean lastY2;
    private boolean wasReady = false;   // used to rumble once when flywheels reach speed

    // ---- Fire gate state ----
    // Measured RPM this loop, and how long each shooter has been inside the fire band.
    private double leftRpmNow, rightRpmNow;        // this loop's actual RPM reading (what everything uses)
    private boolean leftInBand, rightInBand;
    private final ElapsedTime leftSettleTimer  = new ElapsedTime();
    private final ElapsedTime rightSettleTimer = new ElapsedTime();
    private String lastFireBlock = "none yet";
    private final ArrayList<ShotRecord> shotLog = new ArrayList<>();   // newest first
    private int shotCount = 0;

    // Previous button states (for detecting a single press)
    private boolean lastUp, lastDown, lastX, lastRB, lastLB, lastA2, lastB2, lastX2, lastY1, lastLeft2, lastRight2;
    private boolean lastNectarOvr2;   // gamepad2 Back (nectar fire override)
    private boolean lastWindowY2;   // gamepad2 Y (fire window step)
    private boolean servo1Held = false;   // true = override is holding servo1 open (no auto-return)

    private boolean snapping = false;      // heading lock active
    private boolean snapOnTarget = false;  // within tolerance of snap heading
    private boolean holding = false;       // heading hold active
    private double  holdHeading = 0;       // heading (radians) being held

    private final double driveBalance = DRIVE_BALANCE;

    @Override
    public void runOpMode() {
        frontLeft    = hardwareMap.get(DcMotor.class, "FLM");
        frontRight   = hardwareMap.get(DcMotor.class, "FRM");
        backLeft     = hardwareMap.get(DcMotor.class, "BLM");
        backRight    = hardwareMap.get(DcMotor.class, "BRM");
        intakeLeft   = hardwareMap.get(DcMotor.class, "ILM");
        intakeRight  = hardwareMap.get(DcMotor.class, "IRM");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "SLM");
        shooterRight = hardwareMap.get(DcMotorEx.class, "SRM");
        servo1       = hardwareMap.get(Servo.class, "servo_NT");
        servo2       = hardwareMap.get(Servo.class, "servo_PT");
        servo3       = hardwareMap.get(Servo.class, "servo_NC");
        servo4       = hardwareMap.get(Servo.class, "servo_FJ");
        rgbLight     = hardwareMap.get(ServoImplEx.class, "rgblight");
        rgbLight.setPwmEnable();
        rgbLight.setPosition(RGB_OFF);   // intake starts off, so the light starts off

        // Limit switches: plain digital inputs, all feeding one shared count.
        int ls = LIMIT_SWITCH_NAMES.length;
        limitSwitches   = new DigitalChannel[ls];
        limitWasPressed = new boolean[ls];
        limitDebounce   = new ElapsedTime[ls];
        for (int i = 0; i < ls; i++) {
            limitSwitches[i] = hardwareMap.get(DigitalChannel.class, LIMIT_SWITCH_NAMES[i]);
            limitSwitches[i].setMode(DigitalChannel.Mode.INPUT);
            limitDebounce[i] = new ElapsedTime();
        }

        // Color sensors: gain + turn on the sensor's white LED so colors read the same in any lighting.
        int n = COLOR_SENSOR_NAMES.length;
        colorSensors = new NormalizedColorSensor[n];
        sensorColor  = new BallColor[n];
        sensorHue    = new float[n];
        sensorSat    = new float[n];
        sensorDist   = new double[n];
        for (int i = 0; i < n; i++) {
            colorSensors[i] = hardwareMap.get(NormalizedColorSensor.class, COLOR_SENSOR_NAMES[i]);
            colorSensors[i].setGain(COLOR_GAIN);
            if (colorSensors[i] instanceof SwitchableLight) {
                ((SwitchableLight) colorSensors[i]).enableLight(true);
            }
            sensorColor[i] = BallColor.NONE;
        }

        // Bulk reads: read ALL motor data from each hub in one call per loop
        // instead of one call per motor. Makes the loop much faster.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        setupLights();
        setLights(LIGHTS_START_ON);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        imu.resetYaw();   // robot's heading at init = "forward"

        // Left side reversed so "positive" drives forward on all wheels.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Shooters usually face each other, so one spins the opposite way.
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake motors usually face each other too, so one is reversed.
        // If the intake pushes pieces out when it should pull in, flip BOTH.
        // If the two sides fight each other, flip ONE.
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos: direction, then move to home so they start in a known spot.
        // NOTE: the servos will move to home as soon as INIT is pressed.
        servo1.setDirection(SERVO1_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo1.setPosition(degToServoPos(SERVO1_HOME_DEG));
        servo2.setDirection(SERVO2_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo2.setPosition(degToServoPos(SERVO2_HOME_DEG));
        servo3.setDirection(SERVO3_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo3.setPosition(degToServoPos(SERVO3_HOME_DEG));
        servo4.setDirection(SERVO4_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo4.setPosition(degToServoPos(SERVO4_HOME_DEG));

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight, intakeLeft, intakeRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Velocity control: the hub's built-in PID holds the target speed
        // even as the battery drops or a game piece loads the flywheel.
        for (DcMotorEx s : new DcMotorEx[]{shooterLeft, shooterRight}) {
            s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            s.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down, easier on motors
        }
        // Read what the hub has BEFORE changing it (shown on INIT telemetry), then apply ours.
        pidfAsFoundLeft  = pidfText(shooterLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        pidfAsFoundRight = pidfText(shooterRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        if (USE_CUSTOM_PIDF) {
            applyShooterPidf(1.0);
        }
        pidfActiveLeft  = pidfText(shooterLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        pidfActiveRight = pidfText(shooterRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

        // During INIT: keep reading the sensors so you can tune colors before the match starts.
        while (!isStarted() && !isStopRequested()) {
            readColorSensors();
            updateTelemetry();
            sleep(50);
        }

        while (opModeIsActive()) {
            // MANUAL bulk caching: clear once at the top of every loop,
            // or every encoder/velocity read will return stale data.
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            updateShooterSettle();   // must run BEFORE handleToggles so the fire gate uses this loop's RPM
            updateShotLog();
            updateVoltageComp();
            drive();
            handleToggles();
            colorSort();
            countLimitSwitch();
            runMechanisms();
            nectarStrip.update();
            pollenStrip.update();
            readyFeedback();
            updateTelemetry();
        }

        stopAll();
    }

    // ---------------- Shooter PIDF ----------------
    // Sends each shooter its own P, I, D and F. fScale multiplies F (1.0 = as tuned).
    private void applyShooterPidf(double fScale) {
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P_LEFT, SHOOTER_I_LEFT, SHOOTER_D_LEFT, SHOOTER_F_LEFT * fScale));
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P_RIGHT, SHOOTER_I_RIGHT, SHOOTER_D_RIGHT, SHOOTER_F_RIGHT * fScale));
        appliedFScale = fScale;
    }

    // Voltage-scaled F: every VCOMP_UPDATE_SEC, rescale F to the battery. Does nothing unless
    // USE_CUSTOM_PIDF and VOLTAGE_COMP_F are both true. Only talks to the hub when the scale
    // moved by VCOMP_MIN_CHANGE or more, so it doesn't slow the loop.
    private void updateVoltageComp() {
        if (!USE_CUSTOM_PIDF || !VOLTAGE_COMP_F) return;
        if (vcompTimer.seconds() < VCOMP_UPDATE_SEC) return;
        vcompTimer.reset();
        double v = batteryVoltage();
        if (v < 7) return;   // bad reading, keep what we have
        double scale = Math.max(VCOMP_MIN_SCALE, Math.min(VCOMP_MAX_SCALE, PIDF_TUNED_VOLTS / v));
        if (Math.abs(scale - appliedFScale) / appliedFScale < VCOMP_MIN_CHANGE) return;
        applyShooterPidf(scale);
        pidfActiveLeft  = String.format("P %.4f  I %.4f  D %.4f  F %.4f",
                SHOOTER_P_LEFT, SHOOTER_I_LEFT, SHOOTER_D_LEFT, SHOOTER_F_LEFT * scale);
        pidfActiveRight = String.format("P %.4f  I %.4f  D %.4f  F %.4f",
                SHOOTER_P_RIGHT, SHOOTER_I_RIGHT, SHOOTER_D_RIGHT, SHOOTER_F_RIGHT * scale);
    }

    private String pidfText(PIDFCoefficients c) {
        return String.format("P %.4f  I %.4f  D %.4f  F %.4f", c.p, c.i, c.d, c.f);
    }

    // ---------------- Field-centric mecanum drive ----------------
    private void drive() {
        double y  = shapeStick(-gamepad1.left_stick_y);
        double x  = shapeStick( gamepad1.left_stick_x);
        double rx = shapeStick( gamepad1.right_stick_x);

        // Start button re-zeros heading. Point the robot away from the driver first.
        if (gamepad1.start) {
            imu.resetYaw();
            holding = false;   // re-capture the hold heading after a reset
        }

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field-centric: rotate the stick direction by the opposite of the robot's heading,
        // so "stick forward" always means "away from the driver".
        // Robot-centric (FIELD_CENTRIC = false): stick forward = robot's front.
        double driveAngle = FIELD_CENTRIC ? -heading : 0;
        double rotX = x * Math.cos(driveAngle) - y * Math.sin(driveAngle);
        double rotY = x * Math.sin(driveAngle) + y * Math.cos(driveAngle);

        rotX *= 1.1;  // counters imperfect strafing

        double scale = gamepad1.left_bumper ? SLOW_MODE : 1.0;
        x = rotX * scale;
        y = rotY * scale;

        // ---- Snap-to-angle ----
        // X toggles the heading lock ON/OFF. Touching the turn stick cancels it.
        boolean xBtn = gamepad1.x;
        if (xBtn && !lastX) {
            snapping = !snapping;
        }
        lastX = xBtn;
        if (Math.abs(rx) > 0) {
            snapping = false;   // driver took over turning
        }

        boolean translating = Math.abs(x) + Math.abs(y) > 0;

        if (snapping) {
            rx = snapTurnPower(heading);   // full strength, not reduced by slow mode
            holding = false;
        } else if (Math.abs(rx) > 0) {
            rx *= scale;                   // driver is turning: no correction
            holding = false;
        } else if (HEADING_HOLD && translating) {
            // Driver isn't turning: lock the current heading and correct any drift/curve.
            if (!holding) {
                holdHeading = heading;
                holding = true;
            }
            rx = holdTurnPower(heading);
        } else {
            rx = 0;
            holding = false;               // robot is sitting still
        }

        // ---- Shake (gamepad2 D-pad right) ----
        // Overrides driving: pure strafe that flips direction every SHAKE_FLIP_SEC.
        boolean y2 = gamepad2.dpad_right;   // SHAKE button
        if (y2 && !lastY2 && !shaking) {
            shaking = true;
            shakeTimer.reset();
            snapping = false;
            holding = false;
        }
        lastY2 = y2;
        if (shaking) {
            double t = shakeTimer.seconds();
            if (t >= SHAKE_TIME_SEC) {
                shaking = false;
            } else {
                double s = ((int) (t / SHAKE_FLIP_SEC)) % 2 == 0 ? SHAKE_POWER : -SHAKE_POWER;
                frontLeft.setPower(s);
                backLeft.setPower(-s);
                frontRight.setPower(-s);
                backRight.setPower(s);
                return;
            }
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Drive balance: positive slows the LEFT side, negative slows the RIGHT side.
        double leftScale  = 1.0 - Math.max(0,  driveBalance);
        double rightScale = 1.0 - Math.max(0, -driveBalance);

        frontLeft.setPower ((y + x + rx) / denominator * TRIM_FL * leftScale);
        backLeft.setPower  ((y - x + rx) / denominator * TRIM_BL * leftScale);
        frontRight.setPower((y - x - rx) / denominator * TRIM_FR * rightScale);
        backRight.setPower ((y + x - rx) / denominator * TRIM_BR * rightScale);
    }

    // Small proportional correction back to holdHeading while driving straight.
    private double holdTurnPower(double headingRad) {
        double error = AngleUnit.normalizeRadians(holdHeading - headingRad);
        double power = -HOLD_KP * error;   // same sign convention as snap
        return Math.max(-HOLD_MAX_POWER, Math.min(HOLD_MAX_POWER, power));
    }

    // Proportional turn toward SNAP_HEADING_DEG. Returns a turn power for rx.
    private double snapTurnPower(double headingRad) {
        double error = AngleUnit.normalizeRadians(Math.toRadians(SNAP_HEADING_DEG) - headingRad);
        snapOnTarget = Math.abs(Math.toDegrees(error)) < SNAP_TOLERANCE_DEG;
        if (snapOnTarget) return 0;

        // IMU yaw is positive counter-clockwise; positive rx turns clockwise, hence the minus.
        double power = -SNAP_KP * error;
        power = Math.max(-SNAP_MAX_POWER, Math.min(SNAP_MAX_POWER, power));

        // Minimum power so the robot doesn't stall a few degrees short.
        if (Math.abs(power) < SNAP_MIN_POWER) {
            power = Math.signum(power) * SNAP_MIN_POWER;
        }
        return power;
    }

    // ---------------- Toggles (flip only on the press, not while held) ----------------
    private void handleToggles() {
        // Gamepad 1 (driver): intake on the triggers (past halfway counts as a press)
        boolean rt1 = gamepad1.right_trigger > TRIGGER_PRESSED;
        boolean lt1 = gamepad1.left_trigger  > TRIGGER_PRESSED;
        // RPM +/- is on GAMEPAD 1: POLLEN = D-pad up/down, NECTAR = B (up) / A (down).
        // Shooter ON/OFF stays on GAMEPAD 2: POLLEN = D-pad left, NECTAR = B.
        boolean pollenUp   = gamepad1.dpad_up,  pollenDown = gamepad1.dpad_down, pollenOnOff = gamepad2.dpad_left;
        boolean nectarUp   = gamepad1.b,        nectarDown = gamepad1.a,         nectarOnOff = gamepad2.b;
        // Map nectar/pollen onto the actual left/right shooter motors.
        boolean up     = NECTAR_IS_LEFT_SHOOTER ? nectarUp    : pollenUp;      // left motor  RPM +
        boolean down   = NECTAR_IS_LEFT_SHOOTER ? nectarDown  : pollenDown;    // left motor  RPM -
        boolean left2  = NECTAR_IS_LEFT_SHOOTER ? nectarOnOff : pollenOnOff;   // left motor  on/off
        boolean b2     = NECTAR_IS_LEFT_SHOOTER ? pollenUp    : nectarUp;      // right motor RPM +
        boolean x2     = NECTAR_IS_LEFT_SHOOTER ? pollenDown  : nectarDown;    // right motor RPM -
        boolean right2 = NECTAR_IS_LEFT_SHOOTER ? pollenOnOff : nectarOnOff;   // right motor on/off
        // Gamepad 2: servos
        boolean rb = gamepad2.right_bumper;   // servo1 (nectar)
        boolean lb = gamepad2.left_bumper;    // servo2 (pollen)
        boolean a2 = gamepad2.x;              // servo3 (nectar)
        boolean y1 = gamepad1.y;   // servo4 toggle (driver)

        // Intake (gamepad1): RT toggles forward, LT toggles reverse. Pressing one cancels the other.
        if (rt1 && !lastRT1) {
            intakeOn = !intakeOn;
            if (intakeOn) intakeReverse = false;
        }
        if (lt1 && !lastLT1) {
            intakeReverse = !intakeReverse;
            if (intakeReverse) intakeOn = false;
        }
        // Shooter on/off: gamepad2's button only turns its shooter ON.
        // To turn it OFF, gamepad1 must be HOLDING Back when gamepad2 presses.
        boolean offHeld = gamepad1.back;
        if (left2 && !lastLeft2) {
            shooterLeftOn = !offHeld;
        }
        if (right2 && !lastRight2) {
            shooterRightOn = !offHeld;
        }
        // Each fire servo: one press starts its cycle, but ONLY if its shooter is settled (fire gate).
        // Presses during the cycle are ignored.
        if (rb && !lastRB && !servo1Out) {
            if (nectarReadyToFire()) {
                servo1Out = true;
                servo1Timer.reset();
                startShot(true, false);
            } else {
                fireBlocked("servo1 (NECTAR)", true);
            }
        }
        // NECTAR OVERRIDE (gamepad2 Back): TOGGLE. First press opens servo1 with NO RPM check and
        // holds it open; next press closes it. Also takes over a normal timed open already in progress.
        boolean nectarOvr = gamepad2.back;
        if (nectarOvr && !lastNectarOvr2) {
            if (servo1Held) {
                servo1Held = false;
                servo1Out = false;          // close now
            } else {
                if (!servo1Out) startShot(true, true);   // only log a new shot if it wasn't already open
                servo1Held = true;
                servo1Out = true;
            }
        }
        lastNectarOvr2 = nectarOvr;
        // FIRE WINDOW step (gamepad2 Y): +10 per press; anything past 80 wraps to 10.
        // Also used by the ready rumble and shot log.
        boolean windowY2 = gamepad2.y;
        if (windowY2 && !lastWindowY2) {
            double next = fireToleranceRpm + FIRE_TOL_STEP;
            fireToleranceRpm = next > FIRE_TOL_MAX ? FIRE_TOL_MIN : next;
        }
        lastWindowY2 = windowY2;
        // Servo3 is NOT RPM-gated: a press always starts its cycle.
        if (a2 && !lastA2 && !servo3Up) {
            servo3Up = true;
            servo3Timer.reset();
        }
        if (lb && !lastLB && !servo2Out) {
            if (pollenReadyToFire()) {
                servo2Out = true;
                servo2Timer.reset();
                startShot(false, false);
            } else {
                fireBlocked("servo2 (POLLEN)", false);
            }
        }
        // Servo4: gamepad1 Y flips between open and closed.
        if (y1 && !lastY1) {
            servo4Open = !servo4Open;
        }
        // Servo4 fine-tune: gamepad1 D-pad right = +1 degree, D-pad left = -1 degree.
        // Changes the position servo4 is sitting at right now (UP or DOWN).
        boolean dUp1 = gamepad1.dpad_right, dDown1 = gamepad1.dpad_left;
        double nudge = 0;
        if (dUp1 && !lastDpadUp1)     nudge += SERVO4_STEP_DEG;
        if (dDown1 && !lastDpadDown1) nudge -= SERVO4_STEP_DEG;
        if (nudge != 0) {
            if (servo4Open) servo4OpenDeg = clampDeg(servo4OpenDeg + nudge);
            else            servo4HomeDeg = clampDeg(servo4HomeDeg + nudge);
        }
        lastDpadUp1 = dUp1;
        lastDpadDown1 = dDown1;
        // LEFT shooter motor speed (+/- RPM_STEP). Stops at MIN_RPM / SHOOTER_MAX_RPM.
        if (up && !lastUp) {
            targetRpmLeft = Math.min(targetRpmLeft + RPM_STEP, SHOOTER_MAX_RPM);
        }
        if (down && !lastDown) {
            targetRpmLeft = Math.max(targetRpmLeft - RPM_STEP, MIN_RPM);
        }
        // RIGHT shooter motor speed (+/- RPM_STEP).
        if (b2 && !lastB2) {
            targetRpmRight = Math.min(targetRpmRight + RPM_STEP, SHOOTER_MAX_RPM);
        }
        if (x2 && !lastX2) {
            targetRpmRight = Math.max(targetRpmRight - RPM_STEP, MIN_RPM);
        }

        // Prism lights: gamepad1 right bumper flips them on/off.
        // Only talks to the Prism on the press, so it doesn't slow the loop.
        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !lastRB1) {
            setLights(!lightsOn);
        }
        lastRB1 = rb1;

        // Fire settle time: gamepad2 right stick click = +0.1s, left stick click = -0.1s.
        // Rounded to 0.01s so repeated taps don't drift.
        boolean rsb2 = gamepad2.right_stick_button;
        boolean lsb2 = gamepad2.left_stick_button;
        if (rsb2 && !lastRSB2) fireSettleSec = Math.min(fireSettleSec + FIRE_SETTLE_STEP, FIRE_SETTLE_MAX);
        if (lsb2 && !lastLSB2) fireSettleSec = Math.max(fireSettleSec - FIRE_SETTLE_STEP, FIRE_SETTLE_MIN);
        fireSettleSec = Math.round(fireSettleSec * 100) / 100.0;
        lastRSB2 = rsb2;
        lastLSB2 = lsb2;

        // Limit switch count (first Prism strip): gamepad2 D-pad up steps it by hand,
        // 0 -> 1 -> 2 -> 3 -> 4 -> back to 0. Use it to fix the count or reset it.
        boolean dUp2 = gamepad2.dpad_up;
        if (dUp2 && !lastDpadUp2) {
            setLimitCount(limitCount >= STRIP1_COLORS.length ? 0 : limitCount + 1);
        }
        lastDpadUp2 = dUp2;

        // Second Prism strip: gamepad2 Start swaps red <-> blue (this is also our ball color).
        boolean start2 = gamepad2.start;
        if (start2 && !lastStart2) {
            strip2Blue = !strip2Blue;
            strip2Solid.setPrimaryColor(strip2Blue ? Color.BLUE : Color.RED);
            // While the siren is running, the new color waits and shows when the siren ends.
            if (lightsOn && !alertActive) {
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_1, strip2Solid);
            }
        }
        lastStart2 = start2;

        // ALERT: gamepad2 D-pad down = siren on strip 2 + intake reverse for ALERT_SEC.
        // Pressing it again while it's running does nothing.
        boolean dDown2 = gamepad2.dpad_down;
        if (dDown2 && !lastDpadDown2) {
            startAlert();
        }
        lastDpadDown2 = dDown2;

        lastRT1 = rt1; lastLT1 = lt1; lastUp = up; lastDown = down; lastRB = rb; lastLB = lb; lastA2 = a2;
        lastB2 = b2; lastX2 = x2; lastY1 = y1; lastLeft2 = left2; lastRight2 = right2;
    }

    // ---------------- Fire gate ----------------
    // Reads both shooter speeds once per loop and tracks how long each has been inside
    // +/- FIRE_TOLERANCE_RPM of its target. Leaving the band (or turning the shooter off,
    // or changing the target) restarts that shooter's settle timer.
    private void updateShooterSettle() {
        leftRpmNow  = Math.abs(ticksPerSecToRpm(shooterLeft.getVelocity()));
        rightRpmNow = Math.abs(ticksPerSecToRpm(shooterRight.getVelocity()));

        // Every single reading has to be in band. One reading outside restarts the timer,
        // so passing the gate means the RPM sat in band the WHOLE settle time.
        leftInBand = shooterLeftOn && Math.abs(leftRpmNow - targetRpmLeft) <= fireToleranceRpm;
        if (!leftInBand) leftSettleTimer.reset();

        rightInBand = shooterRightOn && Math.abs(rightRpmNow - targetRpmRight) <= fireToleranceRpm;
        if (!rightInBand) rightSettleTimer.reset();
    }

    private boolean leftReadyToFire() {
        return !FIRE_GATE_ON || (leftInBand && leftSettleTimer.seconds() >= fireSettleSec);
    }

    private boolean rightReadyToFire() {
        return !FIRE_GATE_ON || (rightInBand && rightSettleTimer.seconds() >= fireSettleSec);
    }

    private boolean nectarReadyToFire() {
        return NECTAR_IS_LEFT_SHOOTER ? leftReadyToFire() : rightReadyToFire();
    }

    private boolean pollenReadyToFire() {
        return NECTAR_IS_LEFT_SHOOTER ? rightReadyToFire() : leftReadyToFire();
    }

    // ---------------- Shot log ----------------
    private class ShotRecord {
        final int number;
        final String name;
        final boolean usesLeft;
        final double target, fireRpm;
        double low, high;
        double backSec = -1;          // seconds from fire until back in band (-1 = not yet)
        boolean droppedOut = false;   // RPM left the +/- band after the shot
        boolean active = true;        // still being watched
        final ElapsedTime timer = new ElapsedTime();

        ShotRecord(int number, String name, boolean usesLeft, double target, double fireRpm) {
            this.number = number;
            this.name = name;
            this.usesLeft = usesLeft;
            this.target = target;
            this.fireRpm = fireRpm;
            low = fireRpm;
            high = fireRpm;
        }

        void update(double rpm) {
            if (!active) return;
            double t = timer.seconds();
            low  = Math.min(low, rpm);
            high = Math.max(high, rpm);
            boolean inBand = Math.abs(rpm - target) <= fireToleranceRpm;
            if (!inBand) {
                droppedOut = true;
                backSec = -1;         // left the band (again): recovery not finished
            } else if (droppedOut && backSec < 0) {
                backSec = t;          // back in band; the LAST return is the one that counts
            }
            if (t >= SHOT_LOG_SEC) active = false;
        }

        String recovery() {
            if (!droppedOut)  return active ? "watching..." : "never left band";
            if (backSec >= 0) return String.format("back in %.2fs", backSec);
            return active ? "recovering..." : String.format("NOT BACK in %.1fs", SHOT_LOG_SEC);
        }
    }

    // Called when servo1 (nectar) or servo2 (pollen) actually fires.
    private void startShot(boolean nectar, boolean override) {
        boolean useLeft = (nectar == NECTAR_IS_LEFT_SHOOTER);
        for (ShotRecord r : shotLog) {
            if (r.usesLeft == useLeft) r.active = false;   // a new shot on this shooter ends the old record
        }
        shotCount++;
        shotLog.add(0, new ShotRecord(shotCount, (nectar ? "NECTAR" : "POLLEN") + (override ? " OVR" : ""), useLeft,
                useLeft ? targetRpmLeft : targetRpmRight,
                useLeft ? leftRpmNow : rightRpmNow));
        while (shotLog.size() > SHOT_LOG_SIZE) shotLog.remove(shotLog.size() - 1);
    }

    // Feeds this loop's RPM into every shot still being watched.
    private void updateShotLog() {
        for (ShotRecord r : shotLog) {
            r.update(r.usesLeft ? leftRpmNow : rightRpmNow);
        }
    }

    // A fire press was refused: short rumble on the operator's gamepad + telemetry reason.
    private void fireBlocked(String which, boolean nectar) {
        boolean useLeft = (nectar == NECTAR_IS_LEFT_SHOOTER);
        boolean on      = useLeft ? shooterLeftOn : shooterRightOn;
        double  rpm     = useLeft ? leftRpmNow    : rightRpmNow;
        double  target  = useLeft ? targetRpmLeft : targetRpmRight;
        lastFireBlock = on
                ? String.format("%s blocked: %.0f / %.0f RPM (need +/-%.0f)", which, rpm, target, fireToleranceRpm)
                : which + " blocked: shooter is OFF";
        gamepad2.rumble(150);
    }

    // Starts the siren on strip 2 + reverse intake for ALERT_SEC. Ignored if already running.
    private void startAlert() {
        if (alertActive) return;
        alertActive = true;
        alertTimer.reset();
        if (lightsOn) {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_1, stripSiren);
        }
    }

    // ---------------- Limit switch press counter ----------------
    // Each NEW press (not while held) adds 1 to the count:
    // 0 OFF -> 1 RED -> 2 GREEN -> 3 BLUE -> 4 WHITE -> 5, 6, 7... (still WHITE).
    // The debounce timer ignores the few milliseconds of contact bounce so one press never counts as 2 or 3.
    // Both switches feed the same count: limit1 press then limit2 press = 2.
    private void countLimitSwitch() {
        for (int i = 0; i < limitSwitches.length; i++) {
            boolean pressed = limitSwitches[i].getState() != LIMIT_PRESSED_IS_LOW;
            if (pressed != limitWasPressed[i] && limitDebounce[i].seconds() >= LIMIT_DEBOUNCE_SEC) {
                if (pressed) {
                    addBall();
                }
                limitWasPressed[i] = pressed;
                limitDebounce[i].reset();
            }
        }
    }

    // One ball counted (from either switch). No upper limit.
    private void addBall() {
        setLimitCount(limitCount + 1);
    }

    // Which STRIP1_COLORS entry the count shows: 1-4 map directly, anything above 4 uses the last (WHITE).
    private int strip1ColorFor(int count) {
        return Math.min(count, STRIP1_COLORS.length) - 1;
    }

    // Sets the count (0-4) and updates the first strip. Only talks to the Prism when it changes.
    private void setLimitCount(int count) {
        if (count == limitCount) return;
        limitCount = count;
        applyStrip1();
    }

    // Count 0 = strip dark (brightness 0). Count 1-4 = that color at normal brightness.
    // Only sends to the Prism if the lights are on; if they're off, it shows when RB turns them back on.
    private void applyStrip1() {
        if (limitCount <= 0) {
            stripSolid.setBrightness(0);
        } else {
            stripSolid.setBrightness(LED_BRIGHTNESS);
            stripSolid.setPrimaryColor(STRIP1_COLORS[strip1ColorFor(limitCount)]);
        }
        if (lightsOn) {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, stripSolid);
        }
    }

    private String limitCountName() {
        return limitCount <= 0 ? "OFF - no balls" : STRIP1_COLOR_NAMES[strip1ColorFor(limitCount)];
    }

    // ---------------- Color sorting ----------------
    // Light 2 RED  -> keep RED,  reject BLUE.
    // Light 2 BLUE -> keep BLUE, reject RED.
    private void colorSort() {
        readColorSensors();
        if (!COLOR_SORT_ON) return;

        BallColor wrong = strip2Blue ? BallColor.RED : BallColor.BLUE;
        for (int i = 0; i < sensorColor.length; i++) {
            if (sensorColor[i] == wrong && !alertActive) {
                lastSortEvent = "rejected " + wrong + " on " + COLOR_SENSOR_NAMES[i];
                startAlert();
                break;
            }
        }
    }

    private void readColorSensors() {
        for (int i = 0; i < colorSensors.length; i++) {
            sensorColor[i] = readBall(i);
        }
    }

    // What color ball (if any) is in front of sensor i.
    private BallColor readBall(int i) {
        NormalizedColorSensor s = colorSensors[i];

        // Sensors with a distance reading (REV Color Sensor V3): only judge color when a ball is close.
        if (s instanceof DistanceSensor) {
            sensorDist[i] = ((DistanceSensor) s).getDistance(DistanceUnit.CM);
            if (!(sensorDist[i] < BALL_DISTANCE_CM)) {   // also catches NaN
                sensorHue[i] = 0;
                sensorSat[i] = 0;
                return BallColor.NONE;
            }
        } else {
            sensorDist[i] = -1;   // this sensor has no distance; saturation decides if a ball is there
        }

        NormalizedRGBA c = s.getNormalizedColors();
        float[] hsv = new float[3];
        android.graphics.Color.colorToHSV(c.toColor(), hsv);   // full name: our Color is the Prism one
        sensorHue[i] = hsv[0];
        sensorSat[i] = hsv[1];

        if (hsv[1] < MIN_SATURATION) return BallColor.NONE;
        float hue = hsv[0];
        if (hue < RED_HUE_LOW || hue > RED_HUE_HIGH)      return BallColor.RED;
        if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX)   return BallColor.BLUE;
        return BallColor.NONE;
    }

    // ---------------- Apply mechanism outputs ----------------
    private void runMechanisms() {
        // ALERT ends after ALERT_SEC: strip 2 goes back to its red/blue color,
        // and the intake goes back to whatever RT/LT had it set to.
        if (alertActive && alertTimer.seconds() >= ALERT_SEC) {
            alertActive = false;
            if (lightsOn) {
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_1, strip2Solid);
            }
        }

        double intakePower = alertActive ? -INTAKE_POWER                 // ALERT overrides the triggers
                : intakeOn    ?  INTAKE_POWER
                : intakeReverse ? -INTAKE_POWER : 0;
        intakeLeft.setPower(intakePower);
        intakeRight.setPower(intakePower);

        // rgblight follows the intake: GREEN = forward, RED = reverse (incl. ALERT), OFF = stopped.
        // Only sends to the light when the color changes.
        double rgb = intakePower > 0 ? RGB_GREEN : intakePower < 0 ? RGB_RED : RGB_OFF;
        if (rgb != rgbLightPos) {
            rgbLight.setPosition(rgb);
            rgbLightPos = rgb;
        }

        // Auto-return: timer-based, so driving and shooters keep working during the wait.
        if (servo1Out && !servo1Held && servo1Timer.seconds() >= SERVO1_HOLD_SEC) {   // held = no auto-return
            servo1Out = false;
        }
        servo1.setPosition(degToServoPos(servo1Out ? SERVO1_TARGET_DEG : SERVO1_HOME_DEG));

        if (servo3Up && servo3Timer.seconds() >= SERVO3_HOLD_SEC) {
            servo3Up = false;
        }
        servo3.setPosition(degToServoPos(servo3Up ? SERVO3_UP_DEG : SERVO3_HOME_DEG));

        if (servo2Out && servo2Timer.seconds() >= SERVO2_HOLD_SEC) {
            servo2Out = false;
        }
        servo2.setPosition(degToServoPos(servo2Out ? SERVO2_TARGET_DEG : SERVO2_HOME_DEG));

        servo4.setPosition(degToServoPos(servo4Open ? servo4OpenDeg : servo4HomeDeg));

        // Each shooter runs on its own. setPower(0) lets a flywheel coast
        // instead of PID-braking to zero.
        if (shooterLeftOn) {
            shooterLeft.setVelocity(rpmToTicksPerSec(targetRpmLeft));
        } else {
            shooterLeft.setPower(0);
        }
        if (shooterRightOn) {
            shooterRight.setVelocity(rpmToTicksPerSec(targetRpmRight));
        } else {
            shooterRight.setPower(0);
        }
    }

    // ---------------- Prism LED lights ----------------
    // Gives each daisy-chained strip its own LED range and animation.
    private void setupLights() {
        PrismAnimations.AnimationBase[] anims = {stripSolid, strip2Solid, nectarStrip.solid, pollenStrip.solid};

        // Strip 1: solid color, starts RED (gamepad2 D-pad up cycles it)
        stripSolid.setPrimaryColor(STRIP1_COLORS[0]);

        // Strip 2: solid RED to start (gamepad2 Start swaps red/blue).
        // The siren below only shows during the ALERT (gamepad2 D-pad down or color sort).
        strip2Solid.setPrimaryColor(Color.RED);

        // Strip 2 ALERT: siren (red / white / blue police lights)
        stripSiren.setPoliceLightsStyle(PrismAnimations.PoliceLights.PoliceLightsStyle.Style1);
        stripSiren.setPeriod(500);          // ms per loop; lower = faster flashing

        // Strips 3 and 4: shooter RPM colors are handled by RpmStrip (updated every loop).

        int start = 0;
        for (int i = 0; i < anims.length; i++) {
            int stop = start + STRIP_LEDS[i] - 1;   // stop index is the LAST LED, inclusive
            anims[i].setStartIndex(start);
            anims[i].setStopIndex(stop);
            anims[i].setBrightness(LED_BRIGHTNESS);
            start = stop + 1;
        }

        // The siren lives on the same LEDs as strip 2's solid color.
        stripSiren.setStartIndex(strip2Solid.getStartIndex());
        stripSiren.setStopIndex(strip2Solid.getStopIndex());
        stripSiren.setBrightness(LED_BRIGHTNESS);

        // Strip 1 shows the starting limit switch count (0 = dark).
        applyStrip1();

        // Strips 3/4: siren + special go on the same LEDs as their solid color.
        nectarStrip.finishSetup();
        pollenStrip.finishSetup();
    }

    // ---- Strips 3 & 4: one shooter's target RPM -> color ----
    // <2200 SIREN, 2200 RED, 2300 ORANGE, 2400 GREEN, 2500 BLUE, 2600 WHITE, >2600 SPECIAL rainbow
    private static final int RPM_LIGHT_SIREN   = -1;   // below the lowest step
    private static final int RPM_LIGHT_SPECIAL = 99;   // above the highest step

    private class RpmStrip {
        final PrismAnimations.Solid        solid   = new PrismAnimations.Solid(Color.GREEN);
        final PrismAnimations.PoliceLights siren   = new PrismAnimations.PoliceLights();
        final PrismAnimations.Rainbow      special = new PrismAnimations.Rainbow();
        final LayerHeight layer;
        final boolean usesLeftShooter;
        int index = -2;   // what it's showing (-2 = not decided yet)

        RpmStrip(LayerHeight layer, boolean usesLeftShooter, int ledCount) {
            this.layer = layer;
            this.usesLeftShooter = usesLeftShooter;
            siren.setPoliceLightsStyle(PrismAnimations.PoliceLights.PoliceLightsStyle.Style1);
            siren.setPeriod(500);
            special.setSpeed(0.9f);           // fast; 0-1, raise for faster
            special.setRepeatAfter(ledCount); // one full rainbow across the strip
        }

        // Call after solid's start/stop indexes are set.
        void finishSetup() {
            for (PrismAnimations.AnimationBase a : new PrismAnimations.AnimationBase[]{siren, special}) {
                a.setStartIndex(solid.getStartIndex());
                a.setStopIndex(solid.getStopIndex());
                a.setBrightness(LED_BRIGHTNESS);
            }
            index = rpmToIndex(targetRpm());
            applyColor();
        }

        double targetRpm() {
            return usesLeftShooter ? targetRpmLeft : targetRpmRight;
        }

        int rpmToIndex(double rpm) {
            if (rpm < RPM_LIGHT_STEPS[0]) return RPM_LIGHT_SIREN;
            if (rpm > RPM_LIGHT_STEPS[RPM_LIGHT_STEPS.length - 1]) return RPM_LIGHT_SPECIAL;
            int i2 = 0;
            for (int i = 0; i < RPM_LIGHT_STEPS.length; i++) {
                if (rpm >= RPM_LIGHT_STEPS[i]) i2 = i;
            }
            return i2;
        }

        void applyColor() {
            if (index >= 0 && index < RPM_LIGHT_COLORS.length) {
                solid.setPrimaryColor(RPM_LIGHT_COLORS[index]);
            }
        }

        // The animation this strip should be running right now.
        PrismAnimations.AnimationBase current() {
            if (index == RPM_LIGHT_SIREN)   return siren;
            if (index == RPM_LIGHT_SPECIAL) return special;
            return solid;
        }

        String name() {
            if (index == RPM_LIGHT_SIREN)   return "SIREN";
            if (index == RPM_LIGHT_SPECIAL) return "SPECIAL (rainbow)";
            return RPM_LIGHT_NAMES[index];
        }

        // Called every loop, but only talks to the Prism when the color actually changes.
        void update() {
            int newIndex = rpmToIndex(targetRpm());
            if (newIndex != index) {
                index = newIndex;
                applyColor();
                if (lightsOn) {
                    prism.insertAndUpdateAnimation(layer, current());
                }
            }
        }
    }

    // Turns the Prism strips on/off. The rgblight is not touched here; it follows the intake.
    private void setLights(boolean on) {
        if (on) {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, stripSolid);
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_1, alertActive ? stripSiren : strip2Solid);
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, nectarStrip.current());
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_3, pollenStrip.current());
        } else {
            prism.clearAllAnimations();
        }
        lightsOn = on;
    }

    // ---------------- Helpers ----------------

    // Converts degrees to the 0.0-1.0 servo position, clamped to the servo's range.
    private double degToServoPos(double deg) {
        return Math.max(0.0, Math.min(1.0, deg / SERVO_RANGE_DEG));
    }

    // Keeps an angle inside the servo's travel (0 to SERVO_RANGE_DEG).
    private double clampDeg(double deg) {
        return Math.max(0.0, Math.min(SERVO_RANGE_DEG, deg));
    }

    // Deadzone + curve: small stick movements = fine control, full stick = full speed.
    // Rescales after the deadzone so there's no "jump" when you leave it.
    private double shapeStick(double v) {
        double mag = Math.abs(v);
        if (mag < STICK_DEADZONE) return 0;
        double scaled = (mag - STICK_DEADZONE) / (1 - STICK_DEADZONE);
        return Math.signum(v) * Math.pow(scaled, STICK_CURVE);
    }
    private double rpmToTicksPerSec(double rpm) {
        return rpm * SHOOTER_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    private boolean shootersReady() {
        if (!shooterLeftOn && !shooterRightOn) return false;
        double l = leftRpmNow;    // same reading as the fire gate
        double r = rightRpmNow;
        // Ready when every shooter that is ON is at its target. Shooters that are OFF are ignored.
        boolean leftOk  = !shooterLeftOn  || Math.abs(l - targetRpmLeft)  <= fireToleranceRpm;
        boolean rightOk = !shooterRightOn || Math.abs(r - targetRpmRight) <= fireToleranceRpm;
        return leftOk && rightOk;
    }

    // ---------------- Driver feedback when flywheels hit target speed ----------------
    // Rumble: one double-buzz the moment every running shooter reaches its target RPM.
    // LED (PS4/PS5 controllers only): RED = off, YELLOW = spinning up, GREEN = ready.
    // Logitech F310 controllers have no rumble or LED; use telemetry instead.
    // (The robot's rgblight is NOT changed here; it follows the intake.)
    private void readyFeedback() {
        boolean ready = shootersReady();

        if (ready && !wasReady) {
            gamepad2.rumbleBlips(2);
        }
        wasReady = ready;

        if (!shooterLeftOn && !shooterRightOn) {
            gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (ready) {
            gamepad2.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    private double minVolts = 99;

    private double batteryVoltage() {
        double v = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double reading = s.getVoltage();
            if (reading > 0) v = Math.min(v, reading);
        }
        return v == Double.POSITIVE_INFINITY ? 0 : v;
    }

    // Telemetry: battery voltage, both shooters' RPM, and the live fire settle time.
    private void updateTelemetry() {
        double volts = batteryVoltage();
        if (volts > 0) minVolts = Math.min(minVolts, volts);
        telemetry.addData("Battery", "%.2f V   (lowest this run: %.2f V)", volts, minVolts);

        double nectarRpm    = NECTAR_IS_LEFT_SHOOTER ? leftRpmNow  : rightRpmNow;
        double pollenRpm    = NECTAR_IS_LEFT_SHOOTER ? rightRpmNow : leftRpmNow;
        double nectarTarget = NECTAR_IS_LEFT_SHOOTER ? targetRpmLeft  : targetRpmRight;
        double pollenTarget = NECTAR_IS_LEFT_SHOOTER ? targetRpmRight : targetRpmLeft;
        telemetry.addData("POLLEN RPM", "%.0f / target %.0f", pollenRpm, pollenTarget);
        telemetry.addData("NECTAR RPM", "%.0f / target %.0f", nectarRpm, nectarTarget);
        telemetry.addData("Fire window", "+/-%.0f RPM", fireToleranceRpm);
        telemetry.addData("Fire settle", "%.2f s", fireSettleSec);
        telemetry.update();
    }

    private void stopAll() {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight, intakeLeft, intakeRight}) {
            m.setPower(0);
        }
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        prism.clearAllAnimations();   // lights off when the OpMode stops
    }
}
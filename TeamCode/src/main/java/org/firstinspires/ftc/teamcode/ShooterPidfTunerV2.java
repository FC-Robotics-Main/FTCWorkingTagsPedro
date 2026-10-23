package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

/*
 * SHOOTER PIDF TUNER V2  -  runs every step of the 2026-10-21 Shooter PIDF Tuning Guide.
 *
 * Only ONE shooter runs at a time (the selected one). The other stays off, so its
 * battery draw can't change the result.
 *
 * Every value lives only while this OpMode runs. When you're done, copy the
 * "PASTE INTO TELEOP" lines from telemetry into DuckTapeRobotics_1_0.java.
 *
 * GAMEPAD 1 (tuning)
 *   B            : switch shooter LEFT (SLM, NECTAR) <-> RIGHT (SRM, POLLEN)  (only when stopped)
 *   A            : start / stop VELOCITY test (PIDF holds the target; times spin-up)
 *   X            : start / stop FULL-POWER benchmark (no speed control; stops itself after 12 s)
 *   Y            : set F from the last full-power test (F = 32767 / top speed in ticks/s)
 *   Back         : toggle AS FOUND (hub's own values) <-> CUSTOM (the values you're editing)
 *   D-pad L / R  : choose which value to edit: P, I, D, F
 *   D-pad U / D  : raise / lower that value by one step (applies to the hub immediately)
 *   LB / RB      : step size /10 / x10
 *   LT / RT      : target RPM -10 / +10 (each shooter keeps its own target)
 *   Left stick button  : starting rule: P = 0.1 x F, I = 0.1 x P, D = 0
 *   Right stick button : P = I = D = 0 (for the F-only step)
 *
 * GAMEPAD 2 (feeding balls, same buttons as the match)
 *   Right bumper : servo1 (NECTAR) fire cycle      Left bumper : servo2 (POLLEN) fire cycle
 *   X            : servo3 (NECTAR) cycle
 *   Y / A        : mark the last shot HIT / MISS (for hit rate)
 *   Back         : reset the shot statistics for both shooters
 *
 * Fire is NOT gated here, so you can see what happens if you fire early.
 * For fair spin-up timing, let the wheel stop completely before each test.
 */
@TeleOp(name = "Shooter PIDF Tuner V2", group = "Tuning")
public class ShooterPidfTunerV2 extends LinearOpMode {

    // ---- Must match DuckTapeRobotics_1_0 ----
    private static final double  TICKS_PER_REV          = 28;
    private static final boolean NECTAR_IS_LEFT_SHOOTER = true;
    // 50, not 40: the hub reports speed in steps of about 43 RPM (20 ticks/s), so a 40 RPM
    // band only accepts the one exact reading and every flicker counts as "out of band".
    private static final double  FIRE_TOLERANCE_RPM     = 50;
    private static final double  FIRE_SETTLE_SEC        = 0.10;

    private static final double  SERVO_RANGE_DEG   = 180;
    private static final double  SERVO1_HOME_DEG   = 15;
    private static final double  SERVO1_TARGET_DEG = 90;
    private static final boolean SERVO1_REVERSED   = true;
    private static final double  SERVO1_HOLD_SEC   = 0.45;
    private static final double  SERVO2_HOME_DEG   = 30;
    private static final double  SERVO2_TARGET_DEG = 110;
    private static final boolean SERVO2_REVERSED   = false;
    private static final double  SERVO2_HOLD_SEC   = 0.3;
    private static final double  SERVO3_HOME_DEG   = 0;
    private static final double  SERVO3_UP_DEG     = 45;
    private static final boolean SERVO3_REVERSED   = false;
    private static final double  SERVO3_HOLD_SEC   = 0.55;

    // Starting values for editing = the TeleOp's current saved values {P, I, D, F}.
    private static final double[] START_LEFT  = {120.0, 0.0, 0.0, 13.0};   // NECTAR tuned 2026-10-21
    private static final double[] START_RIGHT = {180.0, 0.0, 0.0, 14.8};   // POLLEN tuned 2026-10-21

    // Each shooter keeps its own target, set to its match starting speed in DuckTapeRobotics_1_0.
    // B switches shooters and brings that shooter's target with it.
    private static final double[] START_TARGET_RPM = {2350, 2200};   // {LEFT/NECTAR, RIGHT/POLLEN}
    private static final double RPM_STEP = 10;   // LT / RT change per pull (same as the TeleOp)
    private static final double MIN_RPM  = 1000;
    private static final double MAX_RPM  = 6000;

    // Test windows
    // This flywheel's time constant is about 2.3 s, so it needs ~10 s to truly level off.
    // Steady-state error is averaged over 10-12 s; the result line appears at 12 s.
    // Full-power top speed is averaged over 10-12 s; the test stops itself at 12 s.
    private static final double STEADY_FROM_SEC   = 10.0;
    private static final double STEADY_TO_SEC     = 12.0;
    private static final double FULL_TOP_FROM_SEC = 10.0;
    private static final double FULL_TOP_TO_SEC   = 12.0;
    private static final double SHOT_WATCH_SEC    = 4.0;   // POLLEN recovers in ~2 s; watch long enough to time it
    private static final double REST_RPM          = 50;    // "wheel stopped" for fair timing
    private static final double F_FORMULA_TOP     = 32767;

    private static final String[] COEF_NAMES = {"P", "I", "D", "F"};

    // ---- Hardware ----
    private final DcMotorEx[] shooter = new DcMotorEx[2];   // 0 = LEFT (SLM), 1 = RIGHT (SRM)
    private Servo servo1, servo2, servo3;
    private List<LynxModule> hubs;

    // ---- Coefficients ----
    private final double[][] coef = {START_LEFT.clone(), START_RIGHT.clone()};
    private final PIDFCoefficients[] asFound = new PIDFCoefficients[2];
    private final boolean[] custom = {false, false};   // false = hub's AS FOUND values are active
    private final String[] hubReports = {"", ""};      // what the hub says is active (read back)

    // ---- Selection / editing ----
    private int sel = 0;
    private int coefIdx = 3;           // start on F, the first thing to tune
    private double step = 0.1;
    private final double[] savedTarget = START_TARGET_RPM.clone();
    private double targetRpm = START_TARGET_RPM[0];
    private String note = "";

    private enum Mode { IDLE, VELOCITY, FULL_POWER }
    private Mode mode = Mode.IDLE;
    private final ElapsedTime runTimer = new ElapsedTime();

    // ---- Live readings ----
    private final double[] rpm = new double[2];
    private double volts = 0;
    private double startVolts = 0, startRpm = 0;

    // ---- Velocity test measurement ----
    private double reachSec, settleSec, bandStart, peakOver, steadySum;
    private int steadyN;
    private boolean spinSaved, firedDuringRun;
    private final double[] lastReach = {-1, -1};
    private final double[] lastVelVolts = {0, 0};

    // ---- Full-power measurement ----
    private double fullCross, fullSum;
    private int fullN;
    private final double[] topTps = {0, 0};
    private final double[] fullCrossSec = {-1, -1};
    private final double[] fullVolts = {0, 0};

    // ---- Results history (newest first) ----
    private final ArrayList<String> history = new ArrayList<>();
    private static final int HISTORY_SIZE = 4;

    // ---- Servos ----
    private boolean servo1Out, servo2Out, servo3Up;
    private final ElapsedTime servo1Timer = new ElapsedTime();
    private final ElapsedTime servo2Timer = new ElapsedTime();
    private final ElapsedTime servo3Timer = new ElapsedTime();

    // ---- Shots ----
    private class ShotRecord {
        final int shooterIdx;
        final double target, fireRpm;
        double low, high, backSec = -1;
        boolean droppedOut = false, active = true, counted = false;
        final ElapsedTime timer = new ElapsedTime();

        ShotRecord(int shooterIdx, double target, double fireRpm) {
            this.shooterIdx = shooterIdx;
            this.target = target;
            this.fireRpm = fireRpm;
            low = fireRpm;
            high = fireRpm;
        }

        void update(double r) {
            if (!active) return;
            double t = timer.seconds();
            low  = Math.min(low, r);
            high = Math.max(high, r);
            boolean in = Math.abs(r - target) <= FIRE_TOLERANCE_RPM;
            // Recovery = the LAST time it came back into band. A quick blip when the servo opens
            // must not count as "recovered" before the ball has even loaded the wheel.
            if (!in) { droppedOut = true; backSec = -1; }
            else if (droppedOut && backSec < 0) backSec = t;
            if (t >= SHOT_WATCH_SEC) active = false;
        }

        String recovery() {
            if (!droppedOut)  return active ? "watching..." : "never left band";
            if (backSec >= 0) return String.format("back in %.2fs", backSec);
            return active ? "recovering..." : String.format("NOT BACK in %.1fs", SHOT_WATCH_SEC);
        }
    }
    private final ShotRecord[] lastShot = new ShotRecord[2];
    private final ArrayList<ShotRecord> watching = new ArrayList<>();
    private int lastShotShooter = -1;

    // Shot statistics per shooter (for the 20-shot old vs new comparison)
    private final int[] nShots = new int[2], nBack = new int[2], hits = new int[2], misses = new int[2];
    private final double[] sumFire = new double[2], sumFireSq = new double[2], sumBack = new double[2];

    // ---- Button edge detection ----
    private final boolean[] prev = new boolean[32];
    private boolean pressed(int id, boolean now) {
        boolean p = now && !prev[id];
        prev[id] = now;
        return p;
    }

    @Override
    public void runOpMode() {
        shooter[0] = hardwareMap.get(DcMotorEx.class, "SLM");
        shooter[1] = hardwareMap.get(DcMotorEx.class, "SRM");
        shooter[0].setDirection(DcMotorSimple.Direction.FORWARD);
        shooter[1].setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx s : shooter) {
            s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            s.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        // Step 1: read what the hub is using BEFORE we change anything.
        for (int i = 0; i < 2; i++) {
            asFound[i] = shooter[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            readBack(i);
        }

        servo1 = hardwareMap.get(Servo.class, "servo_NT");
        servo2 = hardwareMap.get(Servo.class, "servo_PT");
        servo3 = hardwareMap.get(Servo.class, "servo_NC");
        servo1.setDirection(SERVO1_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo2.setDirection(SERVO2_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo3.setDirection(SERVO3_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo1.setPosition(degToPos(SERVO1_HOME_DEG));
        servo2.setPosition(degToPos(SERVO2_HOME_DEG));
        servo3.setPosition(degToPos(SERVO3_HOME_DEG));

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("SHOOTER PIDF TUNER V2 - press START");
            telemetry.addData("Battery", "%.2f V", readBattery());
            telemetry.addData("AS FOUND left  (SLM)", pidfText(asFound[0]));
            telemetry.addData("AS FOUND right (SRM)", pidfText(asFound[1]));
            telemetry.addLine("Write these down. They are what the TeleOp runs while USE_CUSTOM_PIDF = false.");
            telemetry.addLine("Both shooters start on AS FOUND, so the first A test is your baseline.");
            telemetry.update();
            sleep(50);
        }

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            volts = readBattery();
            for (int i = 0; i < 2; i++) rpm[i] = Math.abs(tpsToRpm(shooter[i].getVelocity()));

            handleButtons();
            runTest();
            updateShots();
            runServos();
            showTelemetry();
        }

        for (DcMotorEx s : shooter) s.setPower(0);
    }

    // ---------------- Buttons ----------------
    private void handleButtons() {
        // Read every button every loop so the edge detection stays in sync.
        boolean a    = pressed(0,  gamepad1.a);
        boolean x    = pressed(1,  gamepad1.x);
        boolean b    = pressed(2,  gamepad1.b);
        boolean y    = pressed(3,  gamepad1.y);
        boolean dl   = pressed(4,  gamepad1.dpad_left);
        boolean dr   = pressed(5,  gamepad1.dpad_right);
        boolean du   = pressed(6,  gamepad1.dpad_up);
        boolean dd   = pressed(7,  gamepad1.dpad_down);
        boolean lb   = pressed(8,  gamepad1.left_bumper);
        boolean rb   = pressed(9,  gamepad1.right_bumper);
        boolean lt   = pressed(10, gamepad1.left_trigger  > 0.5);
        boolean rt   = pressed(11, gamepad1.right_trigger > 0.5);
        boolean lsb  = pressed(12, gamepad1.left_stick_button);
        boolean rsb  = pressed(13, gamepad1.right_stick_button);
        boolean back = pressed(14, gamepad1.back);
        boolean f1   = pressed(15, gamepad2.right_bumper);
        boolean f2   = pressed(16, gamepad2.left_bumper);
        boolean s3   = pressed(17, gamepad2.x);
        boolean hit  = pressed(18, gamepad2.y);
        boolean miss = pressed(19, gamepad2.a);
        boolean rst  = pressed(20, gamepad2.back);

        if (a) {
            if (mode == Mode.VELOCITY) stopTest();
            else { stopTest(); startVelocity(); }
        }
        if (x) {
            if (mode == Mode.FULL_POWER) stopTest();
            else { stopTest(); startFullPower(); }
        }
        if (b) {
            if (mode == Mode.IDLE) { savedTarget[sel] = targetRpm; sel = 1 - sel; targetRpm = savedTarget[sel]; note = ""; }
            else note = "Stop the test (A or X) before switching shooters.";
        }

        double[] c = coef[sel];
        if (dl) coefIdx = (coefIdx + 3) % 4;
        if (dr) coefIdx = (coefIdx + 1) % 4;
        if (du || dd) {
            c[coefIdx] = Math.max(0, round4(c[coefIdx] + (du ? step : -step)));
            applyCustom(sel);
        }
        if (lb) step = Math.max(0.0001, step / 10);
        if (rb) step = Math.min(10, step * 10);

        if (lt || rt) {
            targetRpm = Math.max(MIN_RPM, Math.min(MAX_RPM, targetRpm + (rt ? RPM_STEP : -RPM_STEP)));
            if (mode == Mode.VELOCITY) {
                shooter[sel].setVelocity(rpmToTps(targetRpm));
                resetSpinMeasure();   // a target change starts a fresh measurement
            }
        }
        if (y) {
            if (topTps[sel] > 0) {
                c[3] = round4(F_FORMULA_TOP / topTps[sel]);
                applyCustom(sel);
                note = String.format("F set to %.4f from full-power top speed.", c[3]);
            } else {
                note = "Run the full-power test (X) on this shooter first.";
            }
        }
        if (lsb) {
            c[0] = round4(0.1 * c[3]);
            c[1] = round4(0.1 * c[0]);
            c[2] = 0;
            applyCustom(sel);
            note = "Starting rule applied: P = 0.1 x F, I = 0.1 x P, D = 0.";
        }
        if (rsb) {
            c[0] = 0; c[1] = 0; c[2] = 0;
            applyCustom(sel);
            note = "P, I, D zeroed (F-only step).";
        }
        if (back) {
            if (custom[sel]) applyAsFound(sel);
            else applyCustom(sel);
        }

        int nectar = NECTAR_IS_LEFT_SHOOTER ? 0 : 1;
        int pollen = 1 - nectar;
        if (f1 && !servo1Out) { servo1Out = true; servo1Timer.reset(); startShot(nectar); }
        if (f2 && !servo2Out) { servo2Out = true; servo2Timer.reset(); startShot(pollen); }
        if (s3 && !servo3Up)  { servo3Up = true;  servo3Timer.reset(); }

        if ((hit || miss) && lastShotShooter >= 0) {
            if (hit) hits[lastShotShooter]++; else misses[lastShotShooter]++;
        }
        if (rst) {
            for (int i = 0; i < 2; i++) {
                nShots[i] = 0; nBack[i] = 0; hits[i] = 0; misses[i] = 0;
                sumFire[i] = 0; sumFireSq[i] = 0; sumBack[i] = 0;
            }
            note = "Shot statistics reset.";
        }
    }

    // ---------------- Tests ----------------
    private void startVelocity() {
        mode = Mode.VELOCITY;
        startVolts = volts;
        startRpm = rpm[sel];
        shooter[sel].setVelocity(rpmToTps(targetRpm));
        resetSpinMeasure();
    }

    private void resetSpinMeasure() {
        runTimer.reset();
        reachSec = -1;
        settleSec = -1;
        bandStart = -1;
        peakOver = -1e9;
        steadySum = 0;
        steadyN = 0;
        spinSaved = false;
        firedDuringRun = false;
    }

    private void startFullPower() {
        mode = Mode.FULL_POWER;
        startVolts = volts;
        startRpm = rpm[sel];
        fullCross = -1;
        fullSum = 0;
        fullN = 0;
        shooter[sel].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter[sel].setPower(1.0);
        runTimer.reset();
    }

    private void stopTest() {
        if (mode == Mode.IDLE) return;
        DcMotorEx s = shooter[sel];
        if (mode == Mode.VELOCITY && !spinSaved && runTimer.seconds() > 0.5) saveSpinResult();
        s.setPower(0);
        if (mode == Mode.FULL_POWER) {
            s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (custom[sel]) applyCustom(sel); else applyAsFound(sel);   // make sure the right values are active
        }
        mode = Mode.IDLE;
    }

    private void runTest() {
        double r = rpm[sel];
        double t = runTimer.seconds();
        if (mode == Mode.VELOCITY) {
            if (reachSec < 0 && r >= targetRpm - FIRE_TOLERANCE_RPM) reachSec = t;
            boolean in = Math.abs(r - targetRpm) <= FIRE_TOLERANCE_RPM;
            if (in) {
                if (bandStart < 0) bandStart = t;
                if (settleSec < 0 && t - bandStart >= FIRE_SETTLE_SEC) settleSec = t;   // = when the fire gate would open
            } else {
                bandStart = -1;
            }
            if (reachSec >= 0 && !firedDuringRun && t <= STEADY_TO_SEC) peakOver = Math.max(peakOver, r - targetRpm);
            if (!firedDuringRun && t >= STEADY_FROM_SEC && t <= STEADY_TO_SEC) {
                steadySum += r - targetRpm;
                steadyN++;
            }
            if (!spinSaved && t > STEADY_TO_SEC) saveSpinResult();
        } else if (mode == Mode.FULL_POWER) {
            if (fullCross < 0 && r >= targetRpm) fullCross = t;
            if (t >= FULL_TOP_FROM_SEC) {
                fullSum += Math.abs(shooter[sel].getVelocity());
                fullN++;
            }
            if (t >= FULL_TOP_TO_SEC) {
                topTps[sel] = fullN > 0 ? fullSum / fullN : 0;
                fullCrossSec[sel] = fullCross;
                fullVolts[sel] = startVolts;
                addHistory(String.format("%s FULL PWR @%.1fV%s: to %.0f in %s, top %.0f RPM (%.0f t/s) -> F %.4f",
                        shortName(sel), startVolts, restFlag(), targetRpm, secText(fullCross),
                        tpsToRpm(topTps[sel]), topTps[sel],
                        topTps[sel] > 0 ? F_FORMULA_TOP / topTps[sel] : 0));
                stopTest();
            }
        }
    }

    private void saveSpinResult() {
        spinSaved = true;
        lastReach[sel] = reachSec;
        lastVelVolts[sel] = startVolts;
        String steady = steadyN > 0 ? String.format("%+.0f", steadySum / steadyN) : "n/a";
        String over = peakOver > -1e8 ? String.format("%+.0f", peakOver) : "n/a";
        addHistory(String.format("%s %s @%.1fV%s %.0f RPM: reach %s, CAN FIRE %s, peak %s, steady %s  [%s]",
                shortName(sel), custom[sel] ? "CUSTOM" : "AS FOUND", startVolts, restFlag(), targetRpm,
                secText(reachSec), secText(settleSec), over, steady, activeText(sel)));
    }

    private String restFlag() {
        return startRpm > REST_RPM ? String.format(" (NOT from rest: %.0f)", startRpm) : "";
    }

    // ---------------- Shots ----------------
    private void startShot(int idx) {
        if (lastShot[idx] != null) lastShot[idx].active = false;   // a new shot ends the old record
        double target = (mode == Mode.VELOCITY && idx == sel) ? targetRpm : rpm[idx];
        ShotRecord rec = new ShotRecord(idx, target, rpm[idx]);
        lastShot[idx] = rec;
        watching.add(rec);
        lastShotShooter = idx;
        if (mode == Mode.VELOCITY && idx == sel) firedDuringRun = true;
    }

    private void updateShots() {
        for (int k = watching.size() - 1; k >= 0; k--) {
            ShotRecord r = watching.get(k);
            r.update(rpm[r.shooterIdx]);
            if (!r.active && !r.counted) {
                r.counted = true;
                int i = r.shooterIdx;
                nShots[i]++;
                sumFire[i] += r.fireRpm;
                sumFireSq[i] += r.fireRpm * r.fireRpm;
                if (!r.droppedOut) { nBack[i]++; }                 // never left the band = 0 s recovery
                else if (r.backSec >= 0) { nBack[i]++; sumBack[i] += r.backSec; }
                watching.remove(k);
            }
        }
    }

    private void runServos() {
        if (servo1Out && servo1Timer.seconds() >= SERVO1_HOLD_SEC) servo1Out = false;
        if (servo2Out && servo2Timer.seconds() >= SERVO2_HOLD_SEC) servo2Out = false;
        if (servo3Up  && servo3Timer.seconds() >= SERVO3_HOLD_SEC) servo3Up = false;
        servo1.setPosition(degToPos(servo1Out ? SERVO1_TARGET_DEG : SERVO1_HOME_DEG));
        servo2.setPosition(degToPos(servo2Out ? SERVO2_TARGET_DEG : SERVO2_HOME_DEG));
        servo3.setPosition(degToPos(servo3Up  ? SERVO3_UP_DEG     : SERVO3_HOME_DEG));
    }

    // ---------------- Coefficients ----------------
    private void applyCustom(int i) {
        double[] c = coef[i];
        shooter[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(c[0], c[1], c[2], c[3]));
        custom[i] = true;
        readBack(i);
    }

    private void applyAsFound(int i) {
        shooter[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, asFound[i]);
        custom[i] = false;
        readBack(i);
    }

    private void readBack(int i) {
        hubReports[i] = pidfText(shooter[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    private String activeText(int i) {
        if (!custom[i]) return "AS FOUND " + pidfText(asFound[i]);
        double[] c = coef[i];
        return String.format("P %.4f I %.4f D %.4f F %.4f", c[0], c[1], c[2], c[3]);
    }

    // ---------------- Telemetry ----------------
    private void showTelemetry() {
        telemetry.addData("Shooter", "%s   %s", longName(sel), mode == Mode.IDLE ? "STOPPED"
                : mode == Mode.VELOCITY ? "VELOCITY TEST" : "FULL POWER");
        telemetry.addData("Battery", "%.2f V", volts);
        telemetry.addData("Target", "%.0f RPM (%.0f ticks/s)", targetRpm, rpmToTps(targetRpm));
        telemetry.addData("RPM now", "LEFT %.0f   RIGHT %.0f   (err %+.0f)", rpm[0], rpm[1], rpm[sel] - targetRpm);
        if (mode != Mode.IDLE) telemetry.addData("Test time", "%.2f s", runTimer.seconds());

        telemetry.addData("Active on hub", (custom[sel] ? "CUSTOM  " : "AS FOUND  ") + hubReports[sel]);
        double[] c = coef[sel];
        StringBuilder ed = new StringBuilder();
        for (int k = 0; k < 4; k++) {
            ed.append(k == coefIdx ? ">" : " ").append(COEF_NAMES[k]).append(" ")
                    .append(String.format("%.4f", c[k])).append(k == coefIdx ? "< " : "  ");
        }
        telemetry.addData("Editing", "%s  step %s", ed.toString(), stepText());
        if (!note.isEmpty()) telemetry.addData("Note", note);

        // Headroom: how close the controller gets to the physical limit (guide Step 1).
        if (fullCrossSec[sel] > 0 && lastReach[sel] > 0) {
            double ratio = lastReach[sel] / fullCrossSec[sel];
            telemetry.addData("Headroom", "spin-up %.2fs vs full power %.2fs (x%.2f) -> %s",
                    lastReach[sel], fullCrossSec[sel], ratio,
                    ratio <= 1.3 ? "near the hardware limit; tuning buys little"
                            : "controller is slow; tuning will help");
        }

        // Last shot on this shooter
        ShotRecord s = lastShot[sel];
        if (s != null) {
            telemetry.addData("Last shot", "fired %.0f / %.0f  low %+.0f  high %+.0f  %s",
                    s.fireRpm, s.target, s.low - s.target, s.high - s.target, s.recovery());
        }
        // Shot statistics (guide Step 6)
        if (nShots[sel] > 0) {
            int n = nShots[sel];
            double mean = sumFire[sel] / n;
            double sd = Math.sqrt(Math.max(0, sumFireSq[sel] / n - mean * mean));
            int marked = hits[sel] + misses[sel];
            telemetry.addData("Shot stats", "%d shots  fire RPM %.0f +/-%.0f  recovery %s (%d/%d back)  hits %s",
                    n, mean, sd, nBack[sel] > 0 ? String.format("%.2fs avg", sumBack[sel] / nBack[sel]) : "n/a",
                    nBack[sel], n, marked > 0 ? String.format("%d/%d", hits[sel], marked) : "not marked");
        }

        telemetry.addLine("--- Results (newest first) ---");
        for (String h : history) telemetry.addLine(h);

        telemetry.addLine("--- PASTE INTO TELEOP (DuckTapeRobotics_1_0) ---");
        telemetry.addLine(String.format("SHOOTER_P_LEFT = %.4f; SHOOTER_I_LEFT = %.4f; SHOOTER_D_LEFT = %.4f; SHOOTER_F_LEFT = %.4f;",
                coef[0][0], coef[0][1], coef[0][2], coef[0][3]));
        telemetry.addLine(String.format("SHOOTER_P_RIGHT = %.4f; SHOOTER_I_RIGHT = %.4f; SHOOTER_D_RIGHT = %.4f; SHOOTER_F_RIGHT = %.4f;",
                coef[1][0], coef[1][1], coef[1][2], coef[1][3]));
        double tunedV = lastVelVolts[0] > 0 && lastVelVolts[1] > 0 ? (lastVelVolts[0] + lastVelVolts[1]) / 2
                : Math.max(lastVelVolts[0], lastVelVolts[1]);
        if (tunedV > 0) telemetry.addLine(String.format("PIDF_TUNED_VOLTS = %.1f;  USE_CUSTOM_PIDF = true;", tunedV));

        telemetry.addLine("--- GP1: B shooter | A vel | X full | Y F-from-full | Back asfound/custom");
        telemetry.addLine("    Dpad L/R pick, U/D change | LB/RB step | LT/RT RPM +/-10 | LSB rule | RSB zero PID");
        telemetry.addLine("    GP2: RB nectar fire | LB pollen fire | X servo3 | Y hit | A miss | Back reset");
        telemetry.update();
    }

    private void addHistory(String s) {
        history.add(0, s);
        while (history.size() > HISTORY_SIZE) history.remove(history.size() - 1);
    }

    // ---------------- Helpers ----------------
    private String longName(int i) {
        boolean nectar = (i == 0) == NECTAR_IS_LEFT_SHOOTER;
        return (i == 0 ? "LEFT (SLM)" : "RIGHT (SRM)") + (nectar ? " NECTAR" : " POLLEN");
    }

    private String shortName(int i) {
        boolean nectar = (i == 0) == NECTAR_IS_LEFT_SHOOTER;
        return nectar ? "NECTAR" : "POLLEN";
    }

    private String pidfText(PIDFCoefficients c) {
        return String.format("P %.4f  I %.4f  D %.4f  F %.4f", c.p, c.i, c.d, c.f);
    }

    private String secText(double s) {
        return s >= 0 ? String.format("%.2fs", s) : "never";
    }

    private String stepText() {
        return step >= 1 ? String.format("%.0f", step) : String.format("%.4f", step);
    }

    private double round4(double v) {
        return Math.round(v * 10000.0) / 10000.0;
    }

    private double readBattery() {
        double v = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double r = s.getVoltage();
            if (r > 0) v = Math.min(v, r);
        }
        return v == Double.POSITIVE_INFINITY ? 0 : v;
    }

    private double degToPos(double deg) {
        return Math.max(0.0, Math.min(1.0, deg / SERVO_RANGE_DEG));
    }

    private double rpmToTps(double r) {
        return r * TICKS_PER_REV / 60.0;
    }

    private double tpsToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }
}
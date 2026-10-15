package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * SHOOTER PIDF TUNER: tune the flywheel velocity PIDF live, no rebuilding.
 *
 * STEP 1 - Measure top speed (FULLY CHARGED battery)
 *   HOLD A for 6-8 seconds: shooters run at full power, max speed is recorded.
 *   Release A, then press X: starting P, I, F are calculated for you.
 *   (Skip this if START_* values below are already filled in: they load at init.)
 *
 * STEP 2 - Test and adjust
 *   Y            : toggle shooters ON/OFF at TARGET_RPM
 *   D-pad L/R    : choose which value to edit (P, I, D, F left, F right)
 *   D-pad up/down: raise/lower the selected value
 *   Bumpers      : step size x10 (right) or /10 (left)
 *   B            : clear the shot stats
 *   BACK         : AUTO-F. While spinning and settled (no shooting), adjusts F left
 *                  and F right so each side settles on TARGET_RPM. Press 1-2 times.
 *
 * Feed game pieces through while it runs and watch "Last shot dip" and "Recovery".
 * When it's good, copy the "PASTE INTO DuckTapeRobotics-1.0" line into that file.
 *
 * Uses the same names and directions as DuckTapeRobotics-1.0 (SLM, SRM).
 */
@TeleOp(name = "Shooter PIDF Tuner", group = "Test")
public class ShooterPIDFTuner extends LinearOpMode {

    // Must match DuckTapeRobotics-1.0
    private static final double TICKS_PER_REV = 28;
    private static final double TARGET_RPM    = 2400;

    private static final double AT_SPEED_RPM   = 50;   // spin-up counts as done within this of target
    private static final double SHOT_DROP_RPM  = 100;  // drop this far below settled speed = a shot
    private static final double RECOVERED_RPM  = 25;   // back within this of settled speed = recovered
    private static final double SETTLE_SECONDS = 1.0;  // wait this long AFTER reaching speed before measuring shots

    // ---- Starting values (loaded at init so you don't redo A/X every deploy) ----
    // Set START_F_LEFT / START_F_RIGHT to 0 to start from the SDK defaults instead.
    private static final double START_P       = 9.84;
    private static final double START_I       = 0.0;      // off for now: was causing the overshoot
    private static final double START_D       = 0.0;
    private static final double START_F_LEFT  = 13.06;    // lowered: was settling ~174 RPM high
    private static final double START_F_RIGHT = 17.16;

    private DcMotorEx left, right;

    // P, I, D shared by both shooters; F is separate for each side
    private final double[] k = new double[5];          // P, I, D, F-left, F-right
    private final String[] kName = {"P", "I", "D", "F left", "F right"};
    private int sel = 0;
    private double step = 0.1;

    private boolean running = false;
    private double maxTps = 0;                          // measured top speed, ticks/sec
    private double maxL = 0, maxR = 0;                  // each side's top speed, ticks/sec
    private final ElapsedTime testTimer = new ElapsedTime();

    // Stats
    private final ElapsedTime spinTimer = new ElapsedTime();
    private final ElapsedTime dipTimer  = new ElapsedTime();
    private double spinUpMs = -1, peakRpm = 0;
    private boolean inDip = false;
    private double dipMin = 0, dipBaseline = 0;
    private double lastDipMin = -1, lastDrop = -1, lastRecoveryMs = -1;
    private double settledRpm = 0;        // smoothed steady speed
    private double settledL = 0, settledR = 0;   // per side, for tuning each F
    private double lowestRpm = -1;        // lowest seen since spin-up settled
    private int shots = 0;
    private double worstDrop = -1, worstRecoveryMs = -1;   // biggest dip since B / spin-up

    private boolean lA, lX, lY, lB, lUp, lDown, lLeft, lRight, lLB, lRB, lBack;
    private int autoFCount = 0;

    @Override
    public void runOpMode() {
        left  = hardwareMap.get(DcMotorEx.class, "SLM");
        right = hardwareMap.get(DcMotorEx.class, "SRM");
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Start from whatever the hub is using now (the SDK defaults)
        PIDFCoefficients start = left.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        k[0] = start.p; k[1] = start.i; k[2] = start.d; k[3] = start.f; k[4] = start.f;
        if (START_F_LEFT > 0 && START_F_RIGHT > 0) {
            k[0] = START_P; k[1] = START_I; k[2] = START_D;
            k[3] = START_F_LEFT; k[4] = START_F_RIGHT;
            apply();
        }

        telemetry.addLine("Shooter PIDF Tuner");
        telemetry.addLine("1) Hold A ~3s to measure top speed, then press X");
        telemetry.addLine("2) Y = spin up, D-pad = adjust, feed pieces through");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.a, x = gamepad1.x, y = gamepad1.y, b = gamepad1.b;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean dl = gamepad1.dpad_left, dr = gamepad1.dpad_right;
            boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;

            double tpsL = Math.abs(left.getVelocity());
            double tpsR = Math.abs(right.getVelocity());
            double rpm  = toRpm((tpsL + tpsR) / 2);

            // ---- STEP 1: full-power top-speed test (hold A) ----
            if (a && !lA) {
                maxTps = 0; maxL = 0; maxR = 0;   // fresh measurement each time A is pressed
                testTimer.reset();
                // Raw power for the test. In RUN_USING_ENCODER, setPower(1.0) is capped
                // at the top speed of the motor TYPE in the robot config, which can be wrong.
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (a) {
                maxL = Math.max(maxL, tpsL);
                maxR = Math.max(maxR, tpsR);
                running = false;
                left.setPower(1.0);
                right.setPower(1.0);
                maxTps = Math.max(maxTps, Math.min(tpsL, tpsR));  // slower motor limits both
            } else if (lA) {
                left.setPower(0);
                right.setPower(0);
                left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // back to velocity control
                right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Calculate starting values from the measured top speed
            if (x && !lX && maxTps > 0) {
                // Each side gets its own F from its own top speed
                k[3] = 32767.0 / Math.max(maxL, 1);   // F left
                k[4] = 32767.0 / Math.max(maxR, 1);   // F right
                k[0] = 0.1 * Math.max(k[3], k[4]);    // P
                k[1] = 0.1 * k[0];                    // I
                k[2] = 0;                             // D
                apply();
            }

            // ---- STEP 2: run at target ----
            if (y && !lY) {
                running = !running;
                if (running) {
                    spinTimer.reset();
                    spinUpMs = -1; peakRpm = 0;
                    inDip = false; lowestRpm = -1;
                    shots = 0; worstDrop = -1; worstRecoveryMs = -1;
                    lastDrop = -1; lastRecoveryMs = -1;
                } else {
                    left.setPower(0);
                    right.setPower(0);
                }
            }
            if (running && !a) {
                double tps = TARGET_RPM * TICKS_PER_REV / 60.0;
                left.setVelocity(tps);
                right.setVelocity(tps);
                trackStats(rpm, toRpm(tpsL), toRpm(tpsR));
            }

            // ---- AUTO-F (Back button): scale each side's F so it settles exactly on target ----
            boolean back = gamepad1.back;
            if (back && !lBack && running && !inDip && settledL > 0 && settledR > 0) {
                k[3] *= TARGET_RPM / settledL;
                k[4] *= TARGET_RPM / settledR;
                apply();
                // Re-measure from scratch with the new F values
                spinTimer.reset();
                spinUpMs = -1; peakRpm = 0; lowestRpm = -1;
                settledL = 0; settledR = 0;
                shots = 0; worstDrop = -1; worstRecoveryMs = -1;
                lastDrop = -1; lastRecoveryMs = -1;
                autoFCount++;
            }
            lBack = back;

            // ---- Editing ----
            if (dl && !lLeft)  sel = (sel + 4) % 5;
            if (dr && !lRight) sel = (sel + 1) % 5;
            if (up && !lUp)     { k[sel] += step; apply(); }
            if (down && !lDown) { k[sel] = Math.max(0, k[sel] - step); apply(); }
            if (rb && !lRB) step = Math.min(step * 10, 10);
            if (lb && !lLB) step = Math.max(step / 10, 0.0001);
            if (b && !lB)   {
                lastDipMin = -1; lastDrop = -1; lastRecoveryMs = -1;
                peakRpm = 0; lowestRpm = -1; shots = 0;
                worstDrop = -1; worstRecoveryMs = -1;
            }

            lA = a; lX = x; lY = y; lB = b; lUp = up; lDown = down;
            lLeft = dl; lRight = dr; lLB = lb; lRB = rb;

            // ---- Telemetry ----
            telemetry.addLine("=== VALUES (D-pad L/R select, up/down change) ===");
            for (int i = 0; i < 5; i++) {
                telemetry.addLine((i == sel ? ">> " : "   ") + kName[i] + " = "
                        + String.format("%.4f", k[i]));
            }
            telemetry.addData("Step", "%.4f   (bumpers: x10 / ÷10)", step);
            telemetry.addData("Auto-F (BACK)", "spin up with Y, wait 3 s, press BACK. Done %d time(s)", autoFCount);

            telemetry.addLine();
            telemetry.addLine("=== SPEED ===");
            telemetry.addData("Shooters", a ? "FULL POWER TEST" : running ? "ON" : "off");
            telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
            telemetry.addData("Left / Right RPM", "%.0f / %.0f", toRpm(tpsL), toRpm(tpsR));
            telemetry.addData("Error", "%+.0f RPM", rpm - TARGET_RPM);
            telemetry.addData("Battery", "%.2f V", batteryVoltage());
            telemetry.addData("Config motor type", left.getMotorType().getName());
            if (a) telemetry.addData("Full power test", "%.1f s (hold 6-8 s)", testTimer.seconds());
            telemetry.addData("Top speed L / R", "%.0f / %.0f RPM", toRpm(maxL), toRpm(maxR));
            telemetry.addData("Measured top speed", maxTps > 0
                    ? String.format("%.0f ticks/s (%.0f RPM), press X", maxTps, toRpm(maxTps))
                    : "not measured, hold A");

            telemetry.addLine();
            telemetry.addLine("=== RESULTS ===");
            telemetry.addData("Spin-up time", spinUpMs < 0 ? "-" : String.format("%.0f ms", spinUpMs));
            telemetry.addData("Overshoot", peakRpm > 0
                    ? String.format("%+.0f RPM", peakRpm - TARGET_RPM) : "-");
            telemetry.addData("Settled RPM", settledRpm > 0
                    ? String.format("%.0f  (%+.0f from target)", settledRpm, settledRpm - TARGET_RPM) : "-");
            telemetry.addData("Settled L / R", settledL > 0
                    ? String.format("%.0f (%+.0f) / %.0f (%+.0f)",
                    settledL, settledL - TARGET_RPM, settledR, settledR - TARGET_RPM) : "-");
            telemetry.addData("Lowest RPM", lowestRpm < 0 ? "-" : String.format("%.0f", lowestRpm));
            telemetry.addData("Shots counted", shots);
            telemetry.addData("Last shot dip", lastDrop < 0 ? "-"
                    : String.format("-%.0f RPM (down to %.0f)", lastDrop, lastDipMin));
            telemetry.addData("Recovery", lastRecoveryMs < 0 ? "-"
                    : lastRecoveryMs >= 3000 ? "did NOT recover in 3 s"
                    : String.format("%.0f ms", lastRecoveryMs));
            telemetry.addData("BIGGEST dip", worstDrop < 0 ? "-"
                    : String.format("-%.0f RPM, recovered in %s", worstDrop,
                    worstRecoveryMs >= 3000 ? ">3 s" : String.format("%.0f ms", worstRecoveryMs)));

            telemetry.addLine();
            telemetry.addLine("PASTE INTO DuckTapeRobotics-1.0:");
            telemetry.addLine(String.format("P=%.4f  I=%.4f  D=%.4f", k[0], k[1], k[2]));
            telemetry.addLine(String.format("F_LEFT=%.4f  F_RIGHT=%.4f", k[3], k[4]));
            telemetry.update();
        }

        left.setPower(0);
        right.setPower(0);
    }

    private void trackStats(double rpm, double rpmL, double rpmR) {
        peakRpm = Math.max(peakRpm, rpm);

        if (spinUpMs < 0 && rpm >= TARGET_RPM - AT_SPEED_RPM) {
            spinUpMs = spinTimer.milliseconds();
        }

        // Only start measuring once it has actually reached speed, plus a short settle time
        boolean settled = spinUpMs >= 0
                && spinTimer.milliseconds() > spinUpMs + SETTLE_SECONDS * 1000;
        if (!settled) {
            settledRpm = rpm;
            settledL = rpmL;
            settledR = rpmR;
            return;
        }

        // Smoothed "normal" speed, frozen during a shot so the dip doesn't drag it down
        if (!inDip) {
            settledRpm = settledRpm * 0.95 + rpm * 0.05;
            settledL   = settledL   * 0.95 + rpmL * 0.05;
            settledR   = settledR   * 0.95 + rpmR * 0.05;
        }
        lowestRpm = lowestRpm < 0 ? rpm : Math.min(lowestRpm, rpm);

        // A shot = speed falls noticeably below where it had settled
        if (!inDip && rpm < settledRpm - SHOT_DROP_RPM) {
            inDip = true;
            dipBaseline = settledRpm;
            dipMin = rpm;
            dipTimer.reset();
        }
        if (inDip) {
            dipMin = Math.min(dipMin, rpm);
            boolean recovered = rpm >= dipBaseline - RECOVERED_RPM;
            boolean timedOut  = dipTimer.milliseconds() >= 3000;
            if (recovered || timedOut) {
                inDip = false;
                shots++;
                lastDipMin = dipMin;
                lastDrop = dipBaseline - dipMin;
                lastRecoveryMs = timedOut ? 3000 : dipTimer.milliseconds();
                if (lastDrop > worstDrop) {
                    worstDrop = lastDrop;
                    worstRecoveryMs = lastRecoveryMs;
                }
            }
        }
    }

    private void apply() {
        left.setVelocityPIDFCoefficients(k[0], k[1], k[2], k[3]);
        right.setVelocityPIDFCoefficients(k[0], k[1], k[2], k[4]);
    }

    private double batteryVoltage() {
        double v = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double reading = s.getVoltage();
            if (reading > 0) v = Math.min(v, reading);
        }
        return v == Double.POSITIVE_INFINITY ? 0 : v;
    }

    private double toRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }
}
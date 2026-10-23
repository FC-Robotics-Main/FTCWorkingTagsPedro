package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

/*
 * AUTONOMOUS: fire POLLEN 4 times, SHAKE, fire POLLEN 2 more times, then drive off the wall.
 *
 * WHAT IT DOES
 *   1. Spins the POLLEN shooter (SRM, the right shooter motor) up to START_RPM_RIGHT.
 *   2. Fires FIRST_VOLLEY (4) shots. For each shot: waits until the RPM has been within
 *      FIRE_TOLERANCE_RPM of target for FIRE_SETTLE_SEC (same fire gate as the TeleOp), then opens
 *      the pollen trigger (servo2, "servo_PT") to SERVO2_TARGET_DEG, holds for SERVO2_HOLD_SEC,
 *      and returns it home.
 *   3. SHAKE: same as the TeleOp's gamepad2 D-pad right. The robot strafes left/right, flipping
 *      every SHAKE_FLIP_SEC for SHAKE_TIME_SEC at SHAKE_POWER, to knock stuck balls loose.
 *      The shooter keeps spinning during the shake.
 *   4. Fires SECOND_VOLLEY (2) more shots the same way, then turns the shooter off.
 *   5. Drives BACKWARD 3 inches (MOVE_FORWARD_IN = -3) from the starting spot with Pedro Pathing
 *      and holds there.
 *
 * Names and values match DuckTapeRobotics_1_0 so the two stay in sync. If you change a
 * shooter, servo2, or shake value in the TeleOp, change it here too.
 *
 * Start the robot with its FRONT against the wall; it backs straight away from it.
 * If it drives INTO the wall instead, flip MOVE_FORWARD_IN to 3.
 *
 * Balls must already be in position to feed the pollen shooter (this program only works the
 * trigger servo, not the intake).
 */
@Autonomous(name = "DuckTape-Auto-Pollen-4-Shake-2", group = "Autonomous")
public class DuckTapeAutoPollen4 extends LinearOpMode {

    // ---- Shooter motor setup (same as TeleOp) ----
    private static final double SHOOTER_TICKS_PER_REV = 28;

    // ---- Shooter PIDF (same as TeleOp; RIGHT motor SRM = POLLEN) ----
    private static final boolean USE_CUSTOM_PIDF = true;
    private static final double SHOOTER_P_RIGHT = 180.0;
    private static final double SHOOTER_I_RIGHT = 0.0;
    private static final double SHOOTER_D_RIGHT = 0.0;
    private static final double SHOOTER_F_RIGHT = 13.7;

    // ---- Shooter speed (same as TeleOp) ----
    private static final double START_RPM_RIGHT = 2200;   // POLLEN (right shooter) speed

    // ---- Fire gate (same as TeleOp) ----
    private static final boolean FIRE_GATE_ON       = true;
    private static final double  FIRE_TOLERANCE_RPM = 60;
    private static final double  FIRE_SETTLE_SEC    = 0.10;

    // ---- Servo2 = POLLEN trigger (same as TeleOp) ----
    private static final double  SERVO_RANGE_DEG   = 180;
    private static final double  SERVO2_HOME_DEG   = 30;
    private static final double  SERVO2_TARGET_DEG = 110;
    private static final boolean SERVO2_REVERSED   = false;
    private static final double  SERVO2_HOLD_SEC   = 0.3;

    // ---- Shake (same as TeleOp) ----
    private static final double SHAKE_POWER    = 0.5;   // how hard it shakes (0-1)
    private static final double SHAKE_FLIP_SEC = 0.08;  // time each direction
    private static final double SHAKE_TIME_SEC = 0.64;  // total shake time (8 flips, ends where it started)

    // ---- Autonomous-only settings ----
    private static final int     FIRST_VOLLEY       = 4;     // shots before the shake
    private static final int     SECOND_VOLLEY      = 2;     // shots after the shake
    private static final double  SERVO2_RETURN_SEC  = 0.3;   // time for servo2 to get back home before the next shot
    private static final double  SPINUP_TIMEOUT_SEC = 3.0;   // longest wait for the RPM to settle before each shot
    private static final boolean FIRE_ON_TIMEOUT    = true;  // true = fire anyway after the timeout, false = skip that shot
    private static final double  SHAKE_SETTLE_SEC   = 0.25;  // pause after the shake so balls drop into place

    // ---- Drive off the wall (Pedro Pathing) ----
    private static final double MOVE_FORWARD_IN   = -3;    // inches from the start spot: negative = BACKWARD, positive = forward
    private static final double MOVE_TIMEOUT_SEC  = 2.0;   // give up on the move after this long

    // ---- Hardware (same names as TeleOp) ----
    private DcMotorEx shooterRight;
    private Servo servo2;
    private List<LynxModule> hubs;
    private Follower follower;

    // Pedro poses. Heading 0 = the direction the robot faces at the start.
    private final Pose startPose   = new Pose(72, 72, Math.toRadians(0));
    private final Pose offWallPose = new Pose(72 + MOVE_FORWARD_IN, 72, Math.toRadians(0));

    // ---- State (same names as TeleOp) ----
    private double targetRpmRight = START_RPM_RIGHT;
    private double rightRpmNow;
    private boolean rightInBand;
    private final ElapsedTime rightSettleTimer = new ElapsedTime();

    private int shotsFired = 0;
    private static final int TOTAL_SHOTS = FIRST_VOLLEY + SECOND_VOLLEY;
    private String status = "INIT";

    @Override
    public void runOpMode() {
        shooterRight = hardwareMap.get(DcMotorEx.class, "SRM");
        servo2       = hardwareMap.get(Servo.class, "servo_PT");

        // Bulk reads, cleared every loop (same as TeleOp).
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Pedro follower: drive motors + Pinpoint come from pedroPathing/Constants.java.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Shooter: same direction and velocity setup as TeleOp.
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (USE_CUSTOM_PIDF) {
            shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(SHOOTER_P_RIGHT, SHOOTER_I_RIGHT, SHOOTER_D_RIGHT, SHOOTER_F_RIGHT));
        }

        // Servo2 to home. NOTE: it moves as soon as INIT is pressed.
        servo2.setDirection(SERVO2_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo2.setPosition(degToServoPos(SERVO2_HOME_DEG));

        status = "Ready: POLLEN " + FIRST_VOLLEY + " shots, shake, " + SECOND_VOLLEY
                + " shots, then " + MOVE_FORWARD_IN + " in forward";
        while (!isStarted() && !isStopRequested()) {
            follower.update();
            updateTelemetry();
        }
        if (isStopRequested()) return;

        // Spin up the POLLEN shooter. It stays on through both volleys and the shake.
        shooterRight.setVelocity(rpmToTicksPerSec(targetRpmRight));

        fireShots(FIRST_VOLLEY);
        shake();
        fireShots(SECOND_VOLLEY);

        // Shooting done: shooter coasts down, trigger stays home.
        shooterRight.setPower(0);
        servo2.setPosition(degToServoPos(SERVO2_HOME_DEG));

        driveOffWall();

        status = "DONE: fired " + shotsFired + " of " + TOTAL_SHOTS + ", off the wall";
        while (opModeIsActive()) {
            readShooter();   // keeps follower.update() running so it holds position
            updateTelemetry();
        }
    }

    // ---------------- Shooting ----------------
    // Fires the pollen trigger `count` times, each one gated on the shooter RPM.
    private void fireShots(int count) {
        for (int i = 1; i <= count && opModeIsActive(); i++) {
            int shot = shotsFired + 1;
            status = "Shot " + shot + "/" + TOTAL_SHOTS + ": waiting for RPM";
            boolean ready = waitForPollenReady();

            if (!ready && !FIRE_ON_TIMEOUT) {
                status = "Shot " + shot + ": SKIPPED (RPM never settled)";
                waitSeconds(0.5);
                continue;
            }

            // Open the pollen trigger, hold, return home.
            status = "Shot " + shot + "/" + TOTAL_SHOTS + ": FIRING" + (ready ? "" : " (timeout, RPM not settled)");
            servo2.setPosition(degToServoPos(SERVO2_TARGET_DEG));
            waitSeconds(SERVO2_HOLD_SEC);

            servo2.setPosition(degToServoPos(SERVO2_HOME_DEG));
            shotsFired++;
            status = "Shot " + shot + "/" + TOTAL_SHOTS + ": returning home";
            waitSeconds(SERVO2_RETURN_SEC);
        }
    }

    // ---------------- Shake (same motion as TeleOp gamepad2 D-pad right) ----------------
    // Pure strafe that flips direction every SHAKE_FLIP_SEC for SHAKE_TIME_SEC.
    // Runs through Pedro's drive so the follower and the shake never fight over the motors.
    private void shake() {
        if (!opModeIsActive()) return;
        status = "SHAKING to unstick balls";
        follower.startTeleopDrive();
        ElapsedTime shakeTimer = new ElapsedTime();
        while (opModeIsActive() && shakeTimer.seconds() < SHAKE_TIME_SEC) {
            double t = shakeTimer.seconds();
            double s = ((int) (t / SHAKE_FLIP_SEC)) % 2 == 0 ? SHAKE_POWER : -SHAKE_POWER;
            follower.setTeleOpDrive(0, s, 0, true);   // forward, strafe, turn, robot-centric
            readShooter();
            updateTelemetry();
        }
        follower.setTeleOpDrive(0, 0, 0, true);       // stop the drivetrain
        status = "Shake done, settling";
        waitSeconds(SHAKE_SETTLE_SEC);
    }

    // ---------------- Drive off the wall ----------------
    // Builds the path from wherever the robot is now (the shake may have nudged it)
    // to MOVE_FORWARD_IN inches in front of the start spot, then holds there.
    private void driveOffWall() {
        if (!opModeIsActive()) return;
        status = "Driving " + MOVE_FORWARD_IN + " in off the wall";
        Pose here = follower.getPose();
        PathChain offWall = follower.pathBuilder()
                .addPath(new BezierLine(here, offWallPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        follower.followPath(offWall, true);   // true = hold the end position
        ElapsedTime moveTimer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && moveTimer.seconds() < MOVE_TIMEOUT_SEC) {
            readShooter();
            updateTelemetry();
        }
    }

    // ---------------- Fire gate ----------------
    // Waits until the POLLEN shooter has been within FIRE_TOLERANCE_RPM of target for
    // FIRE_SETTLE_SEC. Returns true when ready, false if SPINUP_TIMEOUT_SEC runs out first.
    private boolean waitForPollenReady() {
        ElapsedTime timeout = new ElapsedTime();
        rightSettleTimer.reset();
        while (opModeIsActive()) {
            readShooter();
            if (!FIRE_GATE_ON || (rightInBand && rightSettleTimer.seconds() >= FIRE_SETTLE_SEC)) {
                return true;
            }
            if (timeout.seconds() >= SPINUP_TIMEOUT_SEC) {
                return false;
            }
            updateTelemetry();
        }
        return false;
    }

    // Once per loop: clears the bulk cache, updates Pedro, reads the POLLEN shooter speed,
    // and updates the settle timer (same fire-gate logic as TeleOp).
    private void readShooter() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        follower.update();
        rightRpmNow = Math.abs(ticksPerSecToRpm(shooterRight.getVelocity()));
        rightInBand = Math.abs(rightRpmNow - targetRpmRight) <= FIRE_TOLERANCE_RPM;
        if (!rightInBand) rightSettleTimer.reset();
    }

    // Waits without freezing telemetry, Pedro, or ignoring Stop.
    private void waitSeconds(double sec) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < sec) {
            readShooter();
            updateTelemetry();
        }
    }

    // ---------------- Helpers (same as TeleOp) ----------------
    private double degToServoPos(double deg) {
        return Math.max(0.0, Math.min(1.0, deg / SERVO_RANGE_DEG));
    }

    private double rpmToTicksPerSec(double rpm) {
        return rpm * SHOOTER_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    private void updateTelemetry() {
        Pose p = follower.getPose();
        telemetry.addData("Status", status);
        telemetry.addData("POLLEN RPM", "%.0f / target %.0f   %s", rightRpmNow, targetRpmRight,
                rightInBand ? "IN BAND" : "WAIT");
        telemetry.addData("Shots fired", "%d / %d", shotsFired, TOTAL_SHOTS);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f deg",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.update();
    }
}
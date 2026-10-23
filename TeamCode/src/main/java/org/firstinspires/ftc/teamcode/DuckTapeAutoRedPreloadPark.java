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
 * AUTONOMOUS: Level 1 solo - Preload and Park (RED)
 *
 * FIELD COORDINATES (Pedro, inches): (0, 0) = A1 corner (audience wall / red side).
 *   x = across the field toward BLUE (columns A -> F), y = toward the REAR WALL (rows 1 -> 6).
 *   One tile = 24 in. The robot SHOOTS FROM THE BACK, so it runs at heading -90 deg the whole
 *   time: BACK (shooter) faces the rear wall, FRONT faces the audience. It never turns.
 *
 * ROUTE (from the "Level 1 solo: Preload and Park (RED)" drawing)
 *   Start  C1, FRONT touching the audience wall, BACK (shooter) facing the rear wall.
 *   Step 1 Fire the POLLEN shooter 4 times at the cell right from the start spot (the robot's
 *          shooting spot is against the wall). No drive before shooting.
 *   Step 2 Back straight off the wall BACK_OFF_WALL_IN (3 in.), strafe STRAIGHT LEFT along the
 *          wall to DODGE_X, then drive BACKWARD straight up column B, well clear of the frame.
 *   Step 3 Park in front of the red loading zone (A5) and hold there.
 *
 * TUNING THE ROUTE
 *   The poses below were measured off the drawing, so they may be an inch or two off.
 *   Adjust the Pose numbers on the field; everything else follows.
 *
 * Shooter + servo2 names and values match DuckTapeRobotics_1_0. If you change them in the
 * TeleOp, change them here too. Balls must already be loaded; this only works the trigger.
 */
@Autonomous(name = "RED L1 Solo: Preload and Park", group = "Autonomous")
public class DuckTapeAutoRedPreloadPark extends LinearOpMode {

    // ---- Shooter motor setup (same as TeleOp) ----
    private static final double SHOOTER_TICKS_PER_REV = 28;

    // ---- Shooter PIDF (same as TeleOp; RIGHT motor SRM = POLLEN) ----
    private static final boolean USE_CUSTOM_PIDF = true;
    private static final double SHOOTER_P_RIGHT = 180.0;
    private static final double SHOOTER_I_RIGHT = 0.0;
    private static final double SHOOTER_D_RIGHT = 0.0;
    private static final double SHOOTER_F_RIGHT = 13.7;

    // ---- Shooter speed (same as TeleOp) ----
    private static final double START_RPM_RIGHT = 2250;   // POLLEN (right shooter) speed

    // ---- Fire gate (same as TeleOp) ----
    private static final boolean FIRE_GATE_ON       = true;
    private static final double  FIRE_TOLERANCE_RPM = 60;
    private static final double  FIRE_SETTLE_SEC    = 0.05;   // RPM must stay in the band this long before a shot

    // ---- Servo2 = POLLEN trigger (same as TeleOp) ----
    private static final double  SERVO_RANGE_DEG   = 180;
    private static final double  SERVO2_HOME_DEG   = 30;
    private static final double  SERVO2_TARGET_DEG = 110;
    private static final boolean SERVO2_REVERSED   = false;
    private static final double  SERVO2_HOLD_SEC   = 0.3;

    // ---- Autonomous-only settings ----
    private static final int     SHOT_COUNT         = 4;     // pollen shots at the cell
    private static final double  SERVO2_RETURN_SEC  = 0.3;   // time for servo2 to get back home before the next shot
    private static final double  SPINUP_TIMEOUT_SEC = 3.0;   // longest wait for the RPM to settle before each shot
    private static final boolean FIRE_ON_TIMEOUT    = true;  // true = fire anyway after the timeout, false = skip that shot
    private static final double  PATH_TIMEOUT_SEC   = 6.0;   // give up on any one path after this long

    // ---- Route poses (inches, Pedro field coordinates) ----
    // The robot SHOOTS FROM THE BACK, so its BACK faces the rear wall (and the cell) the whole time.
    // Heading -90 deg = front faces the audience wall, back faces the rear wall.
    // Every "forward" move on the drawing is the robot driving BACKWARD.
    private static final double BACK_TO_REAR = Math.toRadians(-90);
    private static final double BACK_OFF_WALL_IN = 3;   // inches straight off the wall after shooting, before the route
    private final Pose startPose   = new Pose(59, 9,  BACK_TO_REAR);   // C1, front touching the audience wall = SHOOTING SPOT
    private final Pose offWallPose = new Pose(59, 9 + BACK_OFF_WALL_IN, BACK_TO_REAR);   // backed straight off the wall
    // DODGE_X = the x the robot runs at while it goes around the frame (smaller = farther left,
    // toward the red alliance wall). Don't go much below ~12 or it will rub the red wall.
    // Right after shooting it now strafes STRAIGHT LEFT along the audience wall to DODGE_X first,
    // then goes straight up, so it can't cut the corner into the frame.
    private static final double DODGE_X = 25;
    private final Pose aroundLow   = new Pose(DODGE_X, 9 + BACK_OFF_WALL_IN, BACK_TO_REAR);   // strafed left, still near the wall
    private final Pose aroundHigh  = new Pose(DODGE_X, 91, BACK_TO_REAR);   // straight up column B, past the frame
    private final Pose parkPose    = new Pose(15, 98, BACK_TO_REAR);   // front of the red loading zone (A5)

    // ---- Hardware (same names as TeleOp) ----
    private DcMotorEx shooterRight;
    private Servo servo2;
    private List<LynxModule> hubs;
    private Follower follower;

    private PathChain toPark;

    // ---- State (same names as TeleOp) ----
    private double targetRpmRight = START_RPM_RIGHT;
    private double rightRpmNow;
    private boolean rightInBand;
    private final ElapsedTime rightSettleTimer = new ElapsedTime();

    private int shotsFired = 0;
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
        buildPaths();

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

        status = "Ready: C1, FRONT touching the audience wall, BACK (shooter) toward the cell";
        while (!isStarted() && !isStopRequested()) {
            follower.update();
            updateTelemetry();
        }
        if (isStopRequested()) return;

        // Step 1: shoot right where the robot starts (front touching the audience wall).
        shooterRight.setVelocity(rpmToTicksPerSec(targetRpmRight));
        status = "Step 1: spinning up at the wall";
        fireShots(SHOT_COUNT);

        // Shooting done: shooter coasts down, trigger stays home.
        shooterRight.setPower(0);
        servo2.setPosition(degToServoPos(SERVO2_HOME_DEG));

        // Steps 2 + 3: back straight off the wall, drive around the frame, park in front of the loading zone.
        status = "Steps 2-3: off the wall, around the frame, to park";
        runPath(toPark);

        status = "DONE: fired " + shotsFired + " of " + SHOT_COUNT + ", parked";
        while (opModeIsActive()) {
            readShooter();   // keeps follower.update() running so it holds the park spot
            updateTelemetry();
        }
    }

    // ---------------- Paths ----------------
    private void buildPaths() {
        toPark = follower.pathBuilder()
                .addPath(new BezierLine(startPose, offWallPose))
                .setConstantHeadingInterpolation(BACK_TO_REAR)
                .addPath(new BezierLine(offWallPose, aroundLow))
                .setConstantHeadingInterpolation(BACK_TO_REAR)
                .addPath(new BezierLine(aroundLow, aroundHigh))
                .setConstantHeadingInterpolation(BACK_TO_REAR)
                .addPath(new BezierLine(aroundHigh, parkPose))
                .setConstantHeadingInterpolation(BACK_TO_REAR)
                .build();
    }

    // Follows a path, holds the end point, and returns when done (or after PATH_TIMEOUT_SEC).
    // The follower keeps holding that spot afterward as long as follower.update() runs.
    private void runPath(PathChain path) {
        if (!opModeIsActive()) return;
        follower.followPath(path, true);   // true = hold the end position
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
            readShooter();
            updateTelemetry();
        }
    }

    // ---------------- Shooting ----------------
    // Fires the pollen trigger `count` times, each one gated on the shooter RPM.
    // The follower holds the shooting spot the whole time.
    private void fireShots(int count) {
        for (int i = 1; i <= count && opModeIsActive(); i++) {
            int shot = shotsFired + 1;
            status = "Shot " + shot + "/" + SHOT_COUNT + ": waiting for RPM";
            boolean ready = waitForPollenReady();

            if (!ready && !FIRE_ON_TIMEOUT) {
                status = "Shot " + shot + ": SKIPPED (RPM never settled)";
                waitSeconds(0.5);
                continue;
            }

            status = "Shot " + shot + "/" + SHOT_COUNT + ": FIRING" + (ready ? "" : " (timeout, RPM not settled)");
            servo2.setPosition(degToServoPos(SERVO2_TARGET_DEG));
            waitSeconds(SERVO2_HOLD_SEC);

            servo2.setPosition(degToServoPos(SERVO2_HOME_DEG));
            shotsFired++;
            status = "Shot " + shot + "/" + SHOT_COUNT + ": returning home";
            waitSeconds(SERVO2_RETURN_SEC);
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
        telemetry.addData("Shots fired", "%d / %d", shotsFired, SHOT_COUNT);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f deg",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.update();
    }
}
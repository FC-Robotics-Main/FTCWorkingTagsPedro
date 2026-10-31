package org.firstinspires.ftc.teamcode;

// Pedro Pathing: the follower drives the robot along paths; Pose = a spot on the field plus a heading.
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

// FTC SDK: hubs, op mode base class, motors, servos and timers.
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

// Reads the servo angles the TeleOp saved on the Control Hub.
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

// Our Pedro setup: drive motors, Pinpoint odometry and tuning.
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;
import java.util.List;

/*
 * AUTONOMOUS: RED - Hive + 2 Flowers (version V00, 2026-10-30)
 *
 * WORDS USED HERE
 *   Pose     = a spot on the field (x, y in inches) plus a HEADING (the way the robot's FRONT points).
 *   Heading  = angle in degrees in this file's settings: 0 = front points at the BLUE wall,
 *              90 = REAR wall, -90 = AUDIENCE wall, 180 = RED wall. Pedro wants radians
 *              (pi radians = 180 degrees), so the code converts with Math.toRadians().
 *   Odometry = the Pinpoint and its two small wheels that measure where the robot is, so Pedro knows
 *              when it has reached a pose.
 *   Shovel   = the scoop on the FRONT of the robot, moved by servo4 ("servo_FJ") and servo5 ("FJF2").
 *   Hive     = the goal we shoot POLLEN into. Flower = where the loose pollen balls sit.
 *
 * FIELD COORDINATES (Pedro, inches): (0, 0) = corner of the AUDIENCE wall and the RED wall.
 *   x grows toward the BLUE wall, y grows toward the REAR wall. One tile = 24 in. Tile names A1-F6
 *   match the DuckTape Field Planner (columns A-F from the red wall, rows 1-6 from the audience wall).
 *
 * THE ROUTE
 *    1. Start at the audience wall, FRONT touching the wall, BACK (shooters) toward the hive.
 *    2. Shoot the 4 preloaded POLLEN into the hive.
 *    3. Back up BACK_UP_IN, then turn 180 degrees on the spot (front now faces the rear wall).
 *    4. Drive to just in front of FLOWER 1, turned so the shovel points straight at it.
 *    5. Lower the shovel (servo4 + servo5), start the intake, drive INTAKE_PUSH_IN into the flower,
 *       and keep the intake running INTAKE_EXTRA_SEC to pull in all the balls.
 *    6. Back up out of the flower, raise the shovel, stop the intake.
 *    7. Drive to SHOOT SPOT 2 on the OTHER side of the hive (the hive will have tipped), turned so
 *       the back points at the hive, and shoot the 4 new POLLEN.
 *    8. Turn so the shovel points straight at FLOWER 2 (next to shoot spot 2) and drive up to it.
 *    9. Lower the shovel, intake, drive in, back out, raise the shovel (same as steps 5-6).
 *   10. Drive back to SHOOT SPOT 2 and shoot those 4 POLLEN.
 *   11. Drive to the parking place and hold there.
 *
 * !!! IMPORTANT: THE FIELD POSITIONS ARE NOT MEASURED YET !!!
 *   The hive, the flowers, shoot spot 2 and the parking place are PLACEHOLDER numbers. We do not have
 *   the BIOBUZZ field drawing in the project, so these are not real positions. Driving to wrong
 *   positions can crash the robot into field pieces. So, until ROUTE_POSITIONS_MEASURED is true,
 *   this op mode ONLY does steps 1-2 (shoot from the start spot) and then stops and holds.
 *   To finish it:
 *     a. Put the real positions into the "FIELD POSITIONS" settings below (measure them on the
 *        field, or click them on the DuckTape Field Planner, which uses the same inches).
 *     b. Set ROUTE_POSITIONS_MEASURED = true.
 *     c. Test at low speed first: set MAX_DRIVE_POWER to 0.5.
 *
 * SHOVEL ANGLES COME FROM THE TELEOP
 *   The TeleOp (DuckTapeRobotics-2.8) saves the shovel angles on the Control Hub when the drivers
 *   adjust them. This autonomous reads the same files, so the shovel goes to the same UP and DOWN
 *   angles as in TeleOp. With no saved files it uses the TeleOp defaults (servo4 0 / 87 degrees,
 *   FJF2 8 / 87 degrees).
 *
 * Shooter, trigger (servo2) and intake names and values match DuckTapeRobotics-2.8.
 */
@Autonomous(name = "RED Hive + 2 Flowers V00", group = "Autonomous")
public class DuckTapeAutoRedHiveFlowersV00 extends LinearOpMode {

    // =====================================================================================
    // SAFETY SWITCH. false = only shoot the preload from the start spot, then stop.
    // Set to true ONLY after the FIELD POSITIONS below are measured on the real field.
    // =====================================================================================
    private static final boolean ROUTE_POSITIONS_MEASURED = false;

    // =====================================================================================
    // FIELD POSITIONS (inches, Pedro field coordinates). Only the START is a measured spot (it is
    // the same start as "RED L1 Solo: Preload and Park"). EVERYTHING ELSE IS A PLACEHOLDER.
    // =====================================================================================

    // Start: tile C1, front touching the audience wall, back (shooters) toward the hive.
    private static final double START_X_IN = 59;
    private static final double START_Y_IN = 9;
    private static final double START_HEADING_DEG = -90;

    // How far to back up from the wall after the first shots, before turning 180.
    // "Back up" = drive toward the robot's BACK, which here is toward the rear wall (+y).
    private static final double BACK_UP_IN = 12;

    // The middle of the hive. The code aims the robot's BACK (shooters) at this point from shoot
    // spot 2. PLACEHOLDER (rough guess from the Snap-to-Arc report: about 59 in. from the audience wall).
    private static final double HIVE_X_IN = 60;
    private static final double HIVE_Y_IN = 57;

    // Flower 1: where its balls are, and the spot the robot stops at just in front of it.
    // The robot turns so the shovel points straight from the APPROACH spot at the FLOWER. PLACEHOLDERS.
    private static final double FLOWER1_X_IN          = 100;
    private static final double FLOWER1_Y_IN          = 40;
    private static final double FLOWER1_APPROACH_X_IN = 100;
    private static final double FLOWER1_APPROACH_Y_IN = 22;

    // Shoot spot 2, on the OTHER side of the hive (the hive will have tipped). PLACEHOLDER.
    private static final double SHOOT2_X_IN = 60;
    private static final double SHOOT2_Y_IN = 96;

    // Flower 2, next to shoot spot 2, and the spot just in front of it. PLACEHOLDERS.
    private static final double FLOWER2_X_IN          = 96;
    private static final double FLOWER2_Y_IN          = 110;
    private static final double FLOWER2_APPROACH_X_IN = 80;
    private static final double FLOWER2_APPROACH_Y_IN = 104;

    // Parking place. PLACEHOLDER: the park spot from "RED L1 Solo: Preload and Park" (front of A5).
    private static final double PARK_X_IN = 15;
    private static final double PARK_Y_IN = 98;

    // =====================================================================================
    // DRIVING
    // =====================================================================================

    // How far the robot drives forward from the approach spot INTO the flower with the shovel down.
    private static final double INTAKE_PUSH_IN = 12;

    // Top speed (0 to 1) for normal driving, and the slower speed used while pushing into a flower.
    private static final double MAX_DRIVE_POWER    = 1.0;
    private static final double INTAKE_DRIVE_POWER = 0.4;

    // Give up on any one path or turn after this long, so one stuck move can't use up the whole period.
    private static final double PATH_TIMEOUT_SEC = 5.0;
    private static final double TURN_TIMEOUT_SEC = 2.0;

    // =====================================================================================
    // SHOOTER (RIGHT motor "SRM" = POLLEN) and TRIGGER (servo2 "servo_PT"), same as TeleOp 2.8
    // =====================================================================================
    private static final double SHOOTER_TICKS_PER_REV = 28;
    private static final double SHOOTER_P_RIGHT = 160.0;
    private static final double SHOOTER_I_RIGHT = 0.0;
    private static final double SHOOTER_D_RIGHT = 0.0;
    private static final double SHOOTER_F_RIGHT = 13.7;
    private static final double POLLEN_RPM      = 2400;

    // Fire gate: only fire when the average RPM has been within FIRE_TOLERANCE_RPM of the target for
    // FIRE_SETTLE_SEC. If it never settles within SPINUP_TIMEOUT_SEC, fire anyway.
    private static final double FIRE_TOLERANCE_RPM = 45;
    private static final double FIRE_SETTLE_SEC    = 0.15;
    private static final double SPINUP_TIMEOUT_SEC = 4.0;
    private static final long   RPM_AVG_SAMPLES    = 6;

    // Pollen shots per volley.
    private static final long   SHOTS_PER_VOLLEY = 4;

    // Trigger servo (servo2): degrees for home and fire, and how long it stays out / takes to return.
    private static final double SERVO_RANGE_DEG   = 180;
    private static final double SERVO2_HOME_DEG   = 30;
    private static final double SERVO2_TARGET_DEG = 110;
    private static final double SERVO2_HOLD_SEC   = 0.3;
    private static final double SERVO2_RETURN_SEC = 0.3;

    // =====================================================================================
    // SHOVEL (servo4 "servo_FJ" + servo5 "FJF2") and INTAKE ("ILM" + "IRM"), same as TeleOp 2.8
    // =====================================================================================
    private static final boolean SERVO4_REVERSED       = true;
    private static final boolean SERVO5_REVERSED       = false;
    private static final double  SERVO4_UP_DEFAULT_DEG   = 0;
    private static final double  SERVO4_DOWN_DEFAULT_DEG = 87;
    private static final double  SERVO5_UP_DEFAULT_DEG   = 8;
    private static final double  SERVO5_DOWN_DEFAULT_DEG = 87;

    // Files the TeleOp saves in /sdcard/FIRST/settings/ (servo4's UP angle is not saved; it is fixed).
    private static final String SERVO4_DOWN_FILE = "ducktape_servo4_end.txt";
    private static final String SERVO5_UP_FILE   = "ducktape_fjf2_start.txt";
    private static final String SERVO5_DOWN_FILE = "ducktape_fjf2_end.txt";

    // Time for the shovel to finish moving before the robot drives.
    private static final double SHOVEL_MOVE_SEC = 0.4;

    // Intake power (1.0 = full speed pulling IN) and how long it keeps running after the push.
    private static final double INTAKE_POWER     = 1.0;
    private static final double INTAKE_EXTRA_SEC = 0.8;

    // ---- Hardware ----
    private DcMotorEx shooterRight;
    private DcMotor intakeLeft, intakeRight;
    private Servo servo2, servo4, servo5;
    private List<LynxModule> hubs;
    private Follower follower;

    // ---- Shovel angles in use (loaded from the TeleOp's files) ----
    private double fServo4UpDeg   = SERVO4_UP_DEFAULT_DEG;
    private double fServo4DownDeg = SERVO4_DOWN_DEFAULT_DEG;
    private double fServo5UpDeg   = SERVO5_UP_DEFAULT_DEG;
    private double fServo5DownDeg = SERVO5_DOWN_DEFAULT_DEG;

    // ---- Poses and paths (built in buildPaths) ----
    private Pose startPose, backUpPose, flower1ApproachPose, flower1GrabPose, shoot2Pose;
    private Pose flower2ApproachPose, flower2GrabPose, parkPose;
    private PathChain pathBackUp, pathToFlower1, pathIntoFlower1, pathOutOfFlower1, pathToShoot2;
    private PathChain pathToFlower2, pathIntoFlower2, pathOutOfFlower2, pathBackToShoot2, pathToPark;

    // ---- Shooter state ----
    private double fRpmNow, fRpmAvg;
    private final double[] rpmSamples = new double[(int) RPM_AVG_SAMPLES];
    private long nRpmSampleCount = 0, nRpmSampleIndex = 0;
    private boolean bIsInBand = false;
    private final ElapsedTime settleTimer = new ElapsedTime();

    // ---- Telemetry ----
    private long nShotsFired = 0;
    private boolean bIsShovelDown = false;
    private boolean bIsIntakeOn = false;
    private String strStatus = "INIT";

    /*
     * WHAT: The whole autonomous: sets up the hardware during INIT, then runs the route after START.
     * CALLED FROM: the FTC SDK, when the drivers select this op mode and press INIT.
     */
    @Override
    public void runOpMode() {
        shooterRight = hardwareMap.get(DcMotorEx.class, "SRM");
        intakeLeft   = hardwareMap.get(DcMotor.class, "ILM");
        intakeRight  = hardwareMap.get(DcMotor.class, "IRM");
        servo2       = hardwareMap.get(Servo.class, "servo_PT");
        servo4       = hardwareMap.get(Servo.class, "servo_FJ");

        // tryGet (not get): returns null instead of stopping the op mode if "FJF2" is not configured.
        servo5       = hardwareMap.tryGet(Servo.class, "FJF2");

        // Bulk reads: read each hub once per loop, cleared in readSensors().
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Pedro follower: the drive motors and the Pinpoint come from pedroPathing/Constants.java.
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.setMaxPower(MAX_DRIVE_POWER);

        // Shooter: same direction and speed control as the TeleOp.
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P_RIGHT, SHOOTER_I_RIGHT, SHOOTER_D_RIGHT, SHOOTER_F_RIGHT));

        // Intake: same directions as the TeleOp (if the two sides fight each other, flip ONE there too).
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos. NOTE: they move to their start positions as soon as INIT is pressed.
        servo2.setDirection(Servo.Direction.FORWARD);
        servo2.setPosition(degToServoPos(SERVO2_HOME_DEG));
        if (SERVO4_REVERSED) {
            servo4.setDirection(Servo.Direction.REVERSE);
        } else {
            servo4.setDirection(Servo.Direction.FORWARD);
        }
        if (servo5 != null) {
            if (SERVO5_REVERSED) {
                servo5.setDirection(Servo.Direction.REVERSE);
            } else {
                servo5.setDirection(Servo.Direction.FORWARD);
            }
        }
        loadShovelAngles();
        setShovelDown(false);

        if (ROUTE_POSITIONS_MEASURED) {
            strStatus = "Ready: C1, FRONT touching the audience wall, BACK toward the hive";
        } else {
            strStatus = "Ready (SHOOT ONLY: field positions not measured yet, see the file header)";
        }
        while (!isStarted() && !isStopRequested()) {
            follower.update();
            updateTelemetry();
        }
        if (isStopRequested()) return;

        // Steps 1-2: shoot the 4 preloaded pollen from the start spot.
        shooterRight.setVelocity(rpmToTicksPerSec(POLLEN_RPM));
        strStatus = "Step 2: shooting the preload";
        fireVolley();

        // Safety switch: stop here until the field positions are real.
        if (!ROUTE_POSITIONS_MEASURED) {
            shooterRight.setPower(0);
            strStatus = "DONE (shoot only). Measure the field positions, then set ROUTE_POSITIONS_MEASURED";
            holdUntilEnd();
            return;
        }

        // Step 3: back up, then turn 180 degrees on the spot.
        strStatus = "Step 3: backing up";
        runPath(pathBackUp);
        strStatus = "Step 3: turning 180";
        turnTo(backUpPose.getHeading());

        // Steps 4-6: flower 1.
        strStatus = "Step 4: driving to flower 1";
        runPath(pathToFlower1);
        turnTo(flower1ApproachPose.getHeading());
        collectFromFlower(pathIntoFlower1, pathOutOfFlower1, "flower 1");

        // Step 7: other side of the hive, shoot.
        strStatus = "Step 7: driving to shoot spot 2";
        runPath(pathToShoot2);
        turnTo(shoot2Pose.getHeading());
        strStatus = "Step 7: shooting flower 1's pollen";
        fireVolley();

        // Steps 8-9: flower 2. Turn first so the shovel points straight at it.
        strStatus = "Step 8: driving to flower 2";
        runPath(pathToFlower2);
        turnTo(flower2ApproachPose.getHeading());
        collectFromFlower(pathIntoFlower2, pathOutOfFlower2, "flower 2");

        // Step 10: back to shoot spot 2 and shoot.
        strStatus = "Step 10: back to shoot spot 2";
        runPath(pathBackToShoot2);
        turnTo(shoot2Pose.getHeading());
        strStatus = "Step 10: shooting flower 2's pollen";
        fireVolley();
        shooterRight.setPower(0);

        // Step 11: park.
        strStatus = "Step 11: parking";
        runPath(pathToPark);
        strStatus = "DONE: fired " + nShotsFired + " pollen, parked";
        holdUntilEnd();
    }

    // ---------------- Route building ----------------

    /*
     * WHAT: Makes every pose and path of the route from the FIELD POSITIONS settings. Headings are
     *       worked out here: at a flower approach the FRONT (shovel) points straight at the flower,
     *       and at shoot spot 2 the BACK (shooters) points straight at the hive.
     * CALLED FROM: runOpMode(), once during INIT.
     */
    private void buildPaths() {
        startPose = new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));

        // Backed up toward the rear wall, then turned 180 (front now faces the rear wall).
        double fTurnedHeading = startPose.getHeading() + Math.PI;
        backUpPose = new Pose(START_X_IN, START_Y_IN + BACK_UP_IN, fTurnedHeading);
        Pose backUpBeforeTurn = new Pose(START_X_IN, START_Y_IN + BACK_UP_IN, startPose.getHeading());

        // Flower 1: face it from the approach spot; the grab spot is INTAKE_PUSH_IN farther toward it.
        double fFlower1Heading = headingFromTo(FLOWER1_APPROACH_X_IN, FLOWER1_APPROACH_Y_IN, FLOWER1_X_IN, FLOWER1_Y_IN);
        flower1ApproachPose = new Pose(FLOWER1_APPROACH_X_IN, FLOWER1_APPROACH_Y_IN, fFlower1Heading);
        flower1GrabPose = pushedForward(flower1ApproachPose, INTAKE_PUSH_IN);

        // Shoot spot 2: the BACK points at the hive, so the FRONT points the opposite way (+ 180).
        double fShoot2Heading = headingFromTo(SHOOT2_X_IN, SHOOT2_Y_IN, HIVE_X_IN, HIVE_Y_IN) + Math.PI;
        shoot2Pose = new Pose(SHOOT2_X_IN, SHOOT2_Y_IN, fShoot2Heading);

        // Flower 2: same idea as flower 1.
        double fFlower2Heading = headingFromTo(FLOWER2_APPROACH_X_IN, FLOWER2_APPROACH_Y_IN, FLOWER2_X_IN, FLOWER2_Y_IN);
        flower2ApproachPose = new Pose(FLOWER2_APPROACH_X_IN, FLOWER2_APPROACH_Y_IN, fFlower2Heading);
        flower2GrabPose = pushedForward(flower2ApproachPose, INTAKE_PUSH_IN);

        // Park keeps the shoot-spot heading (no turn on the way).
        parkPose = new Pose(PARK_X_IN, PARK_Y_IN, fShoot2Heading);

        pathBackUp       = straightPath(startPose, backUpBeforeTurn);
        pathToFlower1    = straightPath(backUpPose, flower1ApproachPose);
        pathIntoFlower1  = straightPath(flower1ApproachPose, flower1GrabPose);
        pathOutOfFlower1 = straightPath(flower1GrabPose, flower1ApproachPose);
        pathToShoot2     = straightPath(flower1ApproachPose, shoot2Pose);
        pathToFlower2    = straightPath(shoot2Pose, flower2ApproachPose);
        pathIntoFlower2  = straightPath(flower2ApproachPose, flower2GrabPose);
        pathOutOfFlower2 = straightPath(flower2GrabPose, flower2ApproachPose);
        pathBackToShoot2 = straightPath(flower2ApproachPose, shoot2Pose);
        pathToPark       = straightPath(shoot2Pose, parkPose);
    }

    /*
     * WHAT: Makes a one-leg straight path (a BezierLine) from one pose to another. If the two headings
     *       are the same, the robot keeps that heading the whole way; otherwise it turns smoothly from
     *       the first heading to the second while it drives (Pedro's "linear heading interpolation").
     * CALLED FROM: buildPaths().
     */
    private PathChain straightPath(Pose from, Pose to) {
        if (Math.abs(from.getHeading() - to.getHeading()) < 1e-6) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(from, to))
                    .setConstantHeadingInterpolation(from.getHeading())
                    .build();
        }
        return follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }

    /*
     * WHAT: Returns the heading (radians) of a line from point (fFromX, fFromY) to point (fToX, fToY).
     *       A robot at the first point with this heading has its FRONT pointing straight at the second.
     * CALLED FROM: buildPaths().
     */
    private double headingFromTo(double fFromX, double fFromY, double fToX, double fToY) {
        return Math.atan2(fToY - fFromY, fToX - fFromX);
    }

    /*
     * WHAT: Returns a pose fDistanceIn inches straight ahead of the given pose (in the direction its
     *       FRONT points), with the same heading. Used for "drive forward into the flower".
     * CALLED FROM: buildPaths().
     */
    private Pose pushedForward(Pose pose, double fDistanceIn) {
        double fX = pose.getX() + Math.cos(pose.getHeading()) * fDistanceIn;
        double fY = pose.getY() + Math.sin(pose.getHeading()) * fDistanceIn;
        return new Pose(fX, fY, pose.getHeading());
    }

    // ---------------- Moving ----------------

    /*
     * WHAT: Follows one path and waits until Pedro says it has arrived (isBusy = false means "done
     *       moving"), or until PATH_TIMEOUT_SEC runs out. The robot then holds that spot.
     * CALLED FROM: runOpMode() and collectFromFlower().
     */
    private void runPath(PathChain path) {
        if (!opModeIsActive()) return;
        follower.followPath(path, true);   // true = hold the end position afterward
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < PATH_TIMEOUT_SEC) {
            readSensors();
            updateTelemetry();
        }
    }

    /*
     * WHAT: Turns on the spot to the given heading (radians), the short way round, and waits until the
     *       turn is finished or TURN_TIMEOUT_SEC runs out. Used to make sure the shovel points
     *       straight at a flower (or the back straight at the hive) before doing anything.
     * CALLED FROM: runOpMode().
     */
    private void turnTo(double fHeadingRad) {
        if (!opModeIsActive()) return;
        follower.turnTo(fHeadingRad);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < TURN_TIMEOUT_SEC) {
            readSensors();
            updateTelemetry();
        }
    }

    /*
     * WHAT: Collects the pollen from one flower. The robot is already at the approach spot with the
     *       shovel pointing at the flower. Lowers the shovel, starts the intake, drives in slowly,
     *       runs the intake a little longer, backs out, raises the shovel, stops the intake.
     * CALLED FROM: runOpMode(), once for each flower.
     */
    private void collectFromFlower(PathChain pathIn, PathChain pathOut, String strFlowerName) {
        strStatus = "Lowering the shovel at " + strFlowerName;
        setShovelDown(true);
        setIntake(true);
        waitSeconds(SHOVEL_MOVE_SEC);

        strStatus = "Driving into " + strFlowerName;
        follower.setMaxPower(INTAKE_DRIVE_POWER);
        runPath(pathIn);
        follower.setMaxPower(MAX_DRIVE_POWER);

        strStatus = "Pulling in the pollen at " + strFlowerName;
        waitSeconds(INTAKE_EXTRA_SEC);

        strStatus = "Backing out of " + strFlowerName;
        runPath(pathOut);

        strStatus = "Raising the shovel";
        setShovelDown(false);
        waitSeconds(SHOVEL_MOVE_SEC);
        setIntake(false);
    }

    /*
     * WHAT: Keeps Pedro holding the robot still (and telemetry updating) until the period ends.
     * CALLED FROM: runOpMode(), at the very end.
     */
    private void holdUntilEnd() {
        while (opModeIsActive()) {
            readSensors();
            updateTelemetry();
        }
    }

    // ---------------- Shovel and intake ----------------

    /*
     * WHAT: Moves the shovel (servo4 and servo5 together) DOWN to scoop, or back UP.
     * CALLED FROM: runOpMode() (INIT) and collectFromFlower().
     */
    private void setShovelDown(boolean bIsDown) {
        bIsShovelDown = bIsDown;
        if (bIsDown) {
            servo4.setPosition(degToServoPos(fServo4DownDeg));
            if (servo5 != null) servo5.setPosition(degToServoPos(fServo5DownDeg));
        } else {
            servo4.setPosition(degToServoPos(fServo4UpDeg));
            if (servo5 != null) servo5.setPosition(degToServoPos(fServo5UpDeg));
        }
    }

    /*
     * WHAT: Turns the intake ON (pulling balls in) or OFF.
     * CALLED FROM: collectFromFlower().
     */
    private void setIntake(boolean bIsOn) {
        bIsIntakeOn = bIsOn;
        double fPower = 0;
        if (bIsOn) fPower = INTAKE_POWER;
        intakeLeft.setPower(fPower);
        intakeRight.setPower(fPower);
    }

    /*
     * WHAT: Loads the shovel angles the TeleOp saved, so autonomous uses the same UP and DOWN angles.
     *       Any missing or bad file keeps that angle's default.
     * CALLED FROM: runOpMode(), during INIT.
     */
    private void loadShovelAngles() {
        fServo4DownDeg = readSavedAngle(SERVO4_DOWN_FILE, SERVO4_DOWN_DEFAULT_DEG);
        fServo5UpDeg   = readSavedAngle(SERVO5_UP_FILE, SERVO5_UP_DEFAULT_DEG);
        fServo5DownDeg = readSavedAngle(SERVO5_DOWN_FILE, SERVO5_DOWN_DEFAULT_DEG);
    }

    /*
     * WHAT: Reads one angle (degrees) from a file in /sdcard/FIRST/settings/. Returns fDefaultDeg if the
     *       file is missing, unreadable, or outside 0-180 degrees.
     * CALLED FROM: loadShovelAngles().
     */
    private double readSavedAngle(String strFileName, double fDefaultDeg) {
        try {
            File angleFile = AppUtil.getInstance().getSettingsFile(strFileName);
            if (!angleFile.exists()) return fDefaultDeg;
            double fSavedDeg = Double.parseDouble(ReadWriteFile.readFile(angleFile).trim());
            if (fSavedDeg >= 0 && fSavedDeg <= 180) return fSavedDeg;
        } catch (Exception e) {
            // unreadable file: use the default
        }
        return fDefaultDeg;
    }

    // ---------------- Shooting ----------------

    /*
     * WHAT: Fires SHOTS_PER_VOLLEY pollen. Before each shot it waits for the shooter RPM to settle
     *       (the fire gate), then swings the trigger (servo2) out and back.
     *       Pedro keeps holding the robot in place the whole time.
     * CALLED FROM: runOpMode(), once per volley (3 volleys).
     */
    private void fireVolley() {
        for (long nShot = 1; nShot <= SHOTS_PER_VOLLEY && opModeIsActive(); nShot++) {
            waitForShooterReady();
            servo2.setPosition(degToServoPos(SERVO2_TARGET_DEG));
            waitSeconds(SERVO2_HOLD_SEC);
            servo2.setPosition(degToServoPos(SERVO2_HOME_DEG));
            nShotsFired++;
            waitSeconds(SERVO2_RETURN_SEC);
        }
    }

    /*
     * WHAT: Waits until the average shooter RPM has been within FIRE_TOLERANCE_RPM of POLLEN_RPM for
     *       FIRE_SETTLE_SEC, or until SPINUP_TIMEOUT_SEC runs out (then it fires anyway).
     * CALLED FROM: fireVolley().
     */
    private void waitForShooterReady() {
        ElapsedTime timeout = new ElapsedTime();
        settleTimer.reset();
        while (opModeIsActive() && timeout.seconds() < SPINUP_TIMEOUT_SEC) {
            readSensors();
            if (bIsInBand && settleTimer.seconds() >= FIRE_SETTLE_SEC) return;
            updateTelemetry();
        }
    }

    /*
     * WHAT: Once per loop: clears the bulk cache, updates Pedro (so it keeps driving or holding), and
     *       reads the shooter speed. Keeps a rolling average of the last RPM_AVG_SAMPLES readings,
     *       because single readings jump around in about 43 RPM steps.
     * CALLED FROM: every waiting loop (runPath, turnTo, waitSeconds, waitForShooterReady, holdUntilEnd).
     */
    private void readSensors() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        follower.update();
        fRpmNow = Math.abs(ticksPerSecToRpm(shooterRight.getVelocity()));

        rpmSamples[(int) nRpmSampleIndex] = fRpmNow;
        nRpmSampleIndex = (nRpmSampleIndex + 1) % RPM_AVG_SAMPLES;
        if (nRpmSampleCount < RPM_AVG_SAMPLES) nRpmSampleCount++;
        double fSum = 0;
        for (long i = 0; i < nRpmSampleCount; i++) {
            fSum += rpmSamples[(int) i];
        }
        fRpmAvg = fSum / nRpmSampleCount;

        // The gate needs a full set of readings so one lucky reading can't pass it.
        bIsInBand = nRpmSampleCount == RPM_AVG_SAMPLES && Math.abs(fRpmAvg - POLLEN_RPM) <= FIRE_TOLERANCE_RPM;
        if (!bIsInBand) settleTimer.reset();
    }

    /*
     * WHAT: Waits fSeconds while still updating Pedro and telemetry (so the robot keeps holding still
     *       and Stop still works).
     * CALLED FROM: fireVolley() and collectFromFlower().
     */
    private void waitSeconds(double fSeconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < fSeconds) {
            readSensors();
            updateTelemetry();
        }
    }

    // ---------------- Helpers ----------------

    /*
     * WHAT: Converts a servo angle in degrees to the 0-1 position the servo wants. Clamping keeps it
     *       inside 0-1 (clamping = if it would go past a limit, stop it AT the limit).
     * CALLED FROM: everywhere a servo is moved.
     */
    private double degToServoPos(double fDeg) {
        return Math.max(0.0, Math.min(1.0, fDeg / SERVO_RANGE_DEG));
    }

    /*
     * WHAT: Converts RPM (turns per minute) to encoder ticks per second, the unit setVelocity() wants.
     * CALLED FROM: runOpMode().
     */
    private double rpmToTicksPerSec(double fRpm) {
        return fRpm * SHOOTER_TICKS_PER_REV / 60.0;
    }

    /*
     * WHAT: Converts encoder ticks per second (what getVelocity() returns) to RPM.
     * CALLED FROM: readSensors().
     */
    private double ticksPerSecToRpm(double fTicksPerSec) {
        return fTicksPerSec * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    /*
     * WHAT: Shows the current step, where the robot is, the shooter speed, and the shovel and intake.
     * CALLED FROM: every waiting loop, and the INIT loop.
     */
    private void updateTelemetry() {
        Pose pose = follower.getPose();
        telemetry.addData("Status", strStatus);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f deg",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Pollen RPM", "%.0f avg / target %.0f", fRpmAvg, POLLEN_RPM);
        telemetry.addData("Shots fired", nShotsFired);

        String strShovel = "UP";
        if (bIsShovelDown) strShovel = "DOWN";
        String strIntake = "off";
        if (bIsIntakeOn) strIntake = "ON";
        telemetry.addData("Shovel / intake", strShovel + " / " + strIntake);
        telemetry.addData("Shovel angles", "servo4 %.0f/%.0f  FJF2 %.0f/%.0f (up/down)",
                fServo4UpDeg, fServo4DownDeg, fServo5UpDeg, fServo5DownDeg);
        if (servo5 == null) telemetry.addLine("FJF2 servo OFF: no \"FJF2\" in the robot config");
        if (!ROUTE_POSITIONS_MEASURED) telemetry.addLine("SHOOT ONLY: field positions are placeholders");
        telemetry.update();
    }
}

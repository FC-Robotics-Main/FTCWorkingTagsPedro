package org.firstinspires.ftc.teamcode;

// Pedro Pathing: the follower drives the robot along paths; a Pose = a spot on the field + a heading.
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

// FTC SDK: hubs, the op mode base class, motors, sensors, servos and timers.
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

// Reads the shovel angles the TeleOp saved on the Control Hub.
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

// Our Pedro setup: drive motors (FLM, FRM, BLM, BRM), the Pinpoint odometry, and tuning.
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;
import java.util.List;

/*
 * TWO-FLOWER AUTONOMOUS (version V00, 2026-10-30) - the shared code for RED and BLUE.
 *
 * This file is NOT an op mode by itself (it is "abstract"). The two op modes you pick on the Driver
 * Station are tiny files that only say which alliance they are:
 *   DuckTapeAutoTwoFlowersRedV00  -> "RED Two Flowers V00"
 *   DuckTapeAutoTwoFlowersBlueV00 -> "BLUE Two Flowers V00"
 * Everything else (poses, servo angles, the steps) lives here, so a fix is made once for both.
 *
 * WORDS USED HERE
 *   Pose     = a spot on the field (x, y in inches) plus a HEADING (the way the robot's FRONT points).
 *   Heading  = degrees, counterclockwise (CCW) positive seen from above: 0 = front points at the BLUE
 *              wall, 90 = away from the audience (rear wall), 180 = RED wall, 270 = audience wall.
 *              Pedro wants radians (pi radians = 180 degrees); pose() converts for you.
 *   Odometry = the Pinpoint and its two small wheels, which measure where the robot is, so Pedro knows
 *              when it has arrived. Same settings as pedroPathing/Constants.java.
 *   isBusy   = Pedro's "still moving" flag. When follower.isBusy() is false, the move is finished.
 *   State machine = the autonomous is a list of STATES (steps). Each loop runs only the current state;
 *              when that state is finished it switches to the next one. Every state also has a TIMEOUT:
 *              if it takes too long, the robot gives up on it and moves on, so it can never get stuck.
 *   Shovel   = the scoop on the FRONT, moved by servo_FJ (servo4) and FJF2 (servo5) together.
 *
 * FIELD (Pedro coordinates, inches): (0, 0) = the audience-side corner on the RED wall. x grows toward
 *   the BLUE wall, y grows away from the audience. The field is 141.5 x 141.5 in. inside the walls.
 *   The BLUE poses are the RED poses turned 180 degrees about the middle of the field.
 *
 * ROBOT: FRONT = intake + shovel. BACK = shooters. To SHOOT, the BACK faces the hive. To INTAKE, the
 *   FRONT faces the flower. Poses assume an 18 x 18 in. robot.
 *
 * THE STEPS (state names start with their step number)
 *    1. At START (front against the audience wall, back to the hive): spin up SRM, shoot 4 pollen.
 *    2. Back up to CLEAR_WALL, then turn to FLOWER1_APPROACH's heading (RED 180, BLUE 0).
 *    3. Drive to FLOWER1_APPROACH (flower 1 is on our alliance wall).
 *    4. Lower the shovel.
 *    5. Intake on, drive slowly to FLOWER1_INTAKE until 4 balls are counted (or 1.5 s).
 *    6. Back up to FLOWER1_BACKUP, then raise the shovel.
 *    7. Drive through VIA_TO_SHOOT2 to SHOOT2 (front against the rear wall, back to the tipped hive).
 *    8. Shoot 4 pollen.
 *    9. Drive to FLOWER2_APPROACH and square up so the shovel points straight at flower 2.
 *   10. Lower the shovel, intake on, drive to FLOWER2_INTAKE until 4 balls are counted (or 1.5 s).
 *   11. Back up to FLOWER2_APPROACH, then raise the shovel.
 *   12. Return to SHOOT2 and shoot 4 pollen.
 *   13. Drive to PARK, stop, shooter and intake off.
 *   The shooter (SRM) keeps spinning from step 1 to step 12, so it never has to spin up from zero.
 *
 * "ALL BALLS COLLECTED": counted with the same two limit switches the TeleOp uses ("limit1" and
 *   "limit2"; a press on either one = 1 ball). If they are not in the robot config, the intake steps
 *   simply run for their 1.5 s timeout.
 */
public abstract class DuckTapeAutoTwoFlowersBaseV00 extends OpMode {

    // =====================================================================================
    // POSES (x in., y in., heading degrees). RED column on the left, BLUE column on the right.
    // =====================================================================================
    private static final Pose RED_START             = pose(58.0,    9.0,  270);
    private static final Pose RED_CLEAR_WALL        = pose(58.0,   24.0,  270);
    private static final Pose RED_FLOWER1_APPROACH  = pose(24.0,   47.17, 180);
    private static final Pose RED_FLOWER1_INTAKE    = pose(15.5,   47.17, 180);
    private static final Pose RED_FLOWER1_BACKUP    = pose(26.0,   47.17, 180);
    private static final Pose RED_VIA_TO_SHOOT2     = pose(28.0,  105.0,   90);
    private static final Pose RED_SHOOT2            = pose(59.5,  132.5,   88);
    private static final Pose RED_FLOWER2_APPROACH  = pose(47.17, 116.0,   90);
    private static final Pose RED_FLOWER2_INTAKE    = pose(47.17, 127.0,   90);
    private static final Pose RED_PARK              = pose(10.0,  106.13,  90);

    private static final Pose BLUE_START            = pose(83.5,  132.5,   90);
    private static final Pose BLUE_CLEAR_WALL       = pose(83.5,  117.5,   90);
    private static final Pose BLUE_FLOWER1_APPROACH = pose(117.5,  94.33,   0);
    private static final Pose BLUE_FLOWER1_INTAKE   = pose(126.0,  94.33,   0);
    private static final Pose BLUE_FLOWER1_BACKUP   = pose(115.5,  94.33,   0);
    private static final Pose BLUE_VIA_TO_SHOOT2    = pose(113.5,  36.5,  270);
    private static final Pose BLUE_SHOOT2           = pose(82.0,    9.0,  268);
    private static final Pose BLUE_FLOWER2_APPROACH = pose(94.33,  25.5,  270);
    private static final Pose BLUE_FLOWER2_INTAKE   = pose(94.33,  14.5,  270);
    private static final Pose BLUE_PARK             = pose(131.5,  35.38, 270);

    // =====================================================================================
    // SHOVEL: servo_FJ (servo4, REVERSE) + FJF2 (servo5, FORWARD, mounted mirrored). Degrees.
    // These are the defaults. If the TeleOp saved tuned angles in /sdcard/FIRST/settings/, those win.
    // (servo_FJ's UP angle has no saved file; it is always SERVO4_UP_DEG.)
    // =====================================================================================
    private static final double SERVO4_UP_DEG   = 0;
    private static final double SERVO4_DOWN_DEG = 87;
    private static final double SERVO5_UP_DEG   = 8;
    private static final double SERVO5_DOWN_DEG = 87;
    private static final String SERVO4_DOWN_FILE = "ducktape_servo4_end.txt";
    private static final String SERVO5_UP_FILE   = "ducktape_fjf2_start.txt";
    private static final String SERVO5_DOWN_FILE = "ducktape_fjf2_end.txt";

    // Every servo here has a 180 degree range: position (0 to 1) = degrees / 180.
    private static final double SERVO_RANGE_DEG = 180;

    // =====================================================================================
    // POLLEN SHOOTER (SRM) and FIRE SERVO (servo_PT)
    // =====================================================================================
    private static final double SHOOTER_TICKS_PER_REV = 28;
    private static final double POLLEN_TARGET_RPM     = 2400;
    private static final double SHOOTER_P = 160.0;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 13.7;

    // Fire gate: only fire when SRM has been within FIRE_TOLERANCE_RPM of target for FIRE_SETTLE_SEC.
    // The RPM used is the AVERAGE of the last RPM_AVG_SAMPLES readings, because the hub reports speed
    // in about 43 RPM steps and single readings jump around.
    private static final double FIRE_TOLERANCE_RPM = 45;
    private static final double FIRE_SETTLE_SEC    = 0.30;
    private static final long   RPM_AVG_SAMPLES    = 6;

    // If one shot waits this long and the RPM never settles: FIRE_ON_TIMEOUT true = fire anyway,
    // false = skip that shot (the ball stays in the robot).
    private static final double  SHOT_WAIT_TIMEOUT_SEC = 2.5;
    private static final boolean FIRE_ON_TIMEOUT       = false;

    private static final long   SHOTS_PER_VOLLEY  = 4;
    private static final double FIRE_SERVO_HOME_DEG = 30;
    private static final double FIRE_SERVO_FIRE_DEG = 110;
    private static final double FIRE_SERVO_HOLD_SEC = 0.3;    // stays at FIRE this long
    private static final double FIRE_SERVO_RETURN_SEC = 0.3;  // time to get back home before the next shot

    // =====================================================================================
    // INTAKE (ILM FORWARD + IRM REVERSE) and BALL COUNTER (limit1 + limit2)
    // =====================================================================================
    private static final double   INTAKE_POWER        = 1.0;
    private static final long     BALLS_PER_FLOWER    = 4;
    private static final String[] BALL_SWITCH_NAMES   = {"limit1", "limit2"};
    private static final boolean  SWITCH_PRESSED_IS_LOW = true;   // same wiring as the TeleOp
    private static final double   SWITCH_DEBOUNCE_SEC = 0.02;     // ignore contact bounce for 20 ms

    // =====================================================================================
    // DRIVING SPEEDS and STATE TIMEOUTS (seconds)
    // =====================================================================================
    private static final double MAX_DRIVE_POWER    = 1.0;
    private static final double INTAKE_DRIVE_POWER = 0.4;   // slow while pushing into a flower

    private static final double SHORT_DRIVE_TIMEOUT_SEC = 3.0;   // backing up, small moves
    private static final double DRIVE_TIMEOUT_SEC       = 4.0;   // normal drives
    private static final double LONG_DRIVE_TIMEOUT_SEC  = 6.0;   // flower 1 -> via -> shoot spot 2
    private static final double TURN_TIMEOUT_SEC        = 2.0;
    private static final double SHOVEL_MOVE_SEC         = 0.4;   // time for the shovel to finish moving
    private static final double INTAKE_TIMEOUT_SEC      = 1.5;   // "until all balls are collected (or 1.5 s)"
    private static final double VOLLEY_TIMEOUT_SEC      = 12.0;  // a whole 4-shot volley

    // =====================================================================================
    // THE STATES. Names start with the step number from the list at the top of the file.
    // =====================================================================================
    private enum State {
        STEP1_SHOOT_PRELOAD,
        STEP2_BACK_UP_FROM_WALL,
        STEP2_TURN_TO_FLOWER1,
        STEP3_DRIVE_TO_FLOWER1,
        STEP4_LOWER_SHOVEL,
        STEP5_INTAKE_FLOWER1,
        STEP6_BACK_UP_FROM_FLOWER1,
        STEP6_RAISE_SHOVEL,
        STEP7_DRIVE_TO_SHOOT2,
        STEP8_SHOOT_FLOWER1_POLLEN,
        STEP9_DRIVE_TO_FLOWER2,
        STEP9_SQUARE_UP_TO_FLOWER2,
        STEP10_LOWER_SHOVEL,
        STEP10_INTAKE_FLOWER2,
        STEP11_BACK_UP_FROM_FLOWER2,
        STEP11_RAISE_SHOVEL,
        STEP12_RETURN_TO_SHOOT2,
        STEP12_SHOOT_FLOWER2_POLLEN,
        STEP13_DRIVE_TO_PARK,
        DONE
    }

    // The three parts of one shot inside a volley.
    private enum ShotPhase { WAIT_FOR_RPM, FIRING, RETURNING }

    // ---- Hardware ----
    private DcMotorEx shooter;
    private DcMotor intakeLeft, intakeRight;
    private Servo fireServo, shovelServo4, shovelServo5;
    private DigitalChannel[] ballSwitches;
    private List<LynxModule> hubs;
    private Follower follower;

    // ---- Poses for THIS alliance (picked in init) ----
    private Pose startPose, clearWallPose, flower1ApproachPose, flower1IntakePose, flower1BackupPose;
    private Pose viaToShoot2Pose, shoot2Pose, flower2ApproachPose, flower2IntakePose, parkPose;

    // ---- Paths (built in buildPaths) ----
    private PathChain pathBackUpFromWall, pathToFlower1, pathIntoFlower1, pathBackUpFromFlower1;
    private PathChain pathToShoot2, pathToFlower2, pathIntoFlower2, pathBackUpFromFlower2;
    private PathChain pathReturnToShoot2, pathToPark;

    // ---- Shovel angles in use ----
    private double fServo4DownDeg = SERVO4_DOWN_DEG;
    private double fServo5UpDeg   = SERVO5_UP_DEG;
    private double fServo5DownDeg = SERVO5_DOWN_DEG;

    // ---- State machine ----
    private State state = State.STEP1_SHOOT_PRELOAD;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private double fStateTimeoutSec = 0;
    private String strLastTimeout = "none";

    // ---- Shooter and volley ----
    private final double[] rpmSamples = new double[(int) RPM_AVG_SAMPLES];
    private long nRpmSampleCount = 0, nRpmSampleIndex = 0;
    private double fRpmNow = 0, fRpmAvg = 0;
    private boolean bIsShooterInBand = false;
    private final ElapsedTime inBandTimer = new ElapsedTime();
    private ShotPhase shotPhase = ShotPhase.WAIT_FOR_RPM;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private long nShotsThisVolley = 0, nShotsFiredTotal = 0, nShotsSkipped = 0;

    // ---- Ball counter ----
    private boolean[] bIsSwitchPressed;
    private ElapsedTime[] switchDebounce;
    private long nBallsCounted = 0;   // reset at the start of each intake step

    // ---- Shovel and intake (for telemetry) ----
    private boolean bIsShovelDown = false;
    private boolean bIsIntakeOn = false;

    /*
     * WHAT: Tells the shared code which alliance this op mode is. RED returns false, BLUE returns true.
     * CALLED FROM: init(), once. Written in DuckTapeAutoTwoFlowersRedV00 / DuckTapeAutoTwoFlowersBlueV00.
     */
    protected abstract boolean isBlueAlliance();

    /*
     * WHAT: Makes a Pose from x, y in inches and a heading in DEGREES (Pedro wants radians).
     * CALLED FROM: the pose constants at the top of this file.
     */
    private static Pose pose(double fX, double fY, double fHeadingDeg) {
        return new Pose(fX, fY, Math.toRadians(fHeadingDeg));
    }

    // ================================ INIT ================================

    /*
     * WHAT: Runs once when INIT is pressed: finds the hardware, sets motor and servo directions, sets
     *       the shooter PIDF (the hub forgets it every power cycle), loads the shovel angles, picks
     *       this alliance's poses, builds the paths, and puts the shovel and fire servo in place.
     * CALLED FROM: the FTC SDK, when the drivers press INIT.
     */
    @Override
    public void init() {
        shooter      = hardwareMap.get(DcMotorEx.class, "SRM");
        intakeLeft   = hardwareMap.get(DcMotor.class, "ILM");
        intakeRight  = hardwareMap.get(DcMotor.class, "IRM");
        fireServo    = hardwareMap.get(Servo.class, "servo_PT");
        shovelServo4 = hardwareMap.get(Servo.class, "servo_FJ");
        shovelServo5 = hardwareMap.get(Servo.class, "FJF2");

        // Bulk reads: each hub is read once per loop; the cache is cleared at the top of loop().
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Pollen shooter: velocity control, REVERSE, and the PIDF set fresh on every start.
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));

        // Intake: the two sides face each other, so one runs FORWARD and one REVERSE.
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos. servo_FJ is REVERSE; FJF2 is FORWARD because it is mounted mirrored.
        fireServo.setDirection(Servo.Direction.FORWARD);
        shovelServo4.setDirection(Servo.Direction.REVERSE);
        shovelServo5.setDirection(Servo.Direction.FORWARD);

        setupBallSwitches();
        loadShovelAngles();
        pickAlliancePoses();

        // Pedro follower: drive motors and the Pinpoint come from pedroPathing/Constants.java.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(MAX_DRIVE_POWER);
        buildPaths();

        // NOTE: the servos move as soon as INIT is pressed.
        fireServo.setPosition(degToServoPos(FIRE_SERVO_HOME_DEG));
        setShovelDown(false);
    }

    /*
     * WHAT: Runs over and over between INIT and START: keeps the odometry updating and shows that the
     *       robot is ready and which alliance it is set for.
     * CALLED FROM: the FTC SDK, every loop while waiting for START.
     */
    @Override
    public void init_loop() {
        follower.update();
        String strAlliance = "RED";
        if (isBlueAlliance()) strAlliance = "BLUE";
        telemetry.addData("Ready", strAlliance + ": front against the wall at START, back toward the hive");
        telemetry.addData("Shovel angles", "servo_FJ %.0f/%.0f  FJF2 %.0f/%.0f (up/down)",
                SERVO4_UP_DEG, fServo4DownDeg, fServo5UpDeg, fServo5DownDeg);
        telemetry.addData("Ball counter", ballCounterStatus());
        Pose pose = follower.getPose();
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.getX(), pose.getY(),
                Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }

    /*
     * WHAT: Runs once when START is pressed: spins up the shooter and begins step 1.
     * CALLED FROM: the FTC SDK, when the drivers press START.
     */
    @Override
    public void start() {
        shooter.setVelocity(rpmToTicksPerSec(POLLEN_TARGET_RPM));
        startVolley();
        setState(State.STEP1_SHOOT_PRELOAD);
    }

    // ================================ LOOP ================================

    /*
     * WHAT: Runs about 50+ times a second during the match: reads the hubs, lets Pedro drive, reads the
     *       shooter speed and the ball counter, runs the current state, then shows telemetry.
     * CALLED FROM: the FTC SDK, every loop after START.
     */
    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        follower.update();
        readShooterRpm();
        countBalls();
        runStateMachine();
        showTelemetry();
    }

    /*
     * WHAT: Runs once when the op mode ends (STOP pressed or the period ends): turns everything off.
     * CALLED FROM: the FTC SDK.
     */
    @Override
    public void stop() {
        shooter.setPower(0);
        setIntake(false);
    }

    // ============================ THE STATE MACHINE ============================

    /*
     * WHAT: Runs the CURRENT state once. Each case does its job and, when that job is finished (or its
     *       timeout runs out), switches to the next state with setState().
     * CALLED FROM: loop(), every loop.
     */
    private void runStateMachine() {
        switch (state) {

            case STEP1_SHOOT_PRELOAD:
                if (updateVolley() || isStateTimedOut()) {
                    driveTo(pathBackUpFromWall);
                    setState(State.STEP2_BACK_UP_FROM_WALL);
                }
                break;

            case STEP2_BACK_UP_FROM_WALL:
                if (!follower.isBusy() || isStateTimedOut()) {
                    follower.turnTo(flower1ApproachPose.getHeading());
                    setState(State.STEP2_TURN_TO_FLOWER1);
                }
                break;

            case STEP2_TURN_TO_FLOWER1:
                if (!follower.isBusy() || isStateTimedOut()) {
                    driveTo(pathToFlower1);
                    setState(State.STEP3_DRIVE_TO_FLOWER1);
                }
                break;

            case STEP3_DRIVE_TO_FLOWER1:
                if (!follower.isBusy() || isStateTimedOut()) {
                    setShovelDown(true);
                    setState(State.STEP4_LOWER_SHOVEL);
                }
                break;

            case STEP4_LOWER_SHOVEL:
                if (stateTimer.seconds() >= SHOVEL_MOVE_SEC) {
                    startIntaking(pathIntoFlower1);
                    setState(State.STEP5_INTAKE_FLOWER1);
                }
                break;

            case STEP5_INTAKE_FLOWER1:
                if (nBallsCounted >= BALLS_PER_FLOWER || isStateTimedOut()) {
                    follower.setMaxPower(MAX_DRIVE_POWER);
                    driveTo(pathBackUpFromFlower1);
                    setState(State.STEP6_BACK_UP_FROM_FLOWER1);
                }
                break;

            case STEP6_BACK_UP_FROM_FLOWER1:
                if (!follower.isBusy() || isStateTimedOut()) {
                    setShovelDown(false);
                    setState(State.STEP6_RAISE_SHOVEL);
                }
                break;

            case STEP6_RAISE_SHOVEL:
                if (stateTimer.seconds() >= SHOVEL_MOVE_SEC) {
                    setIntake(false);
                    driveTo(pathToShoot2);
                    setState(State.STEP7_DRIVE_TO_SHOOT2);
                }
                break;

            case STEP7_DRIVE_TO_SHOOT2:
                if (!follower.isBusy() || isStateTimedOut()) {
                    startVolley();
                    setState(State.STEP8_SHOOT_FLOWER1_POLLEN);
                }
                break;

            case STEP8_SHOOT_FLOWER1_POLLEN:
                if (updateVolley() || isStateTimedOut()) {
                    driveTo(pathToFlower2);
                    setState(State.STEP9_DRIVE_TO_FLOWER2);
                }
                break;

            case STEP9_DRIVE_TO_FLOWER2:
                if (!follower.isBusy() || isStateTimedOut()) {
                    follower.turnTo(flower2ApproachPose.getHeading());
                    setState(State.STEP9_SQUARE_UP_TO_FLOWER2);
                }
                break;

            case STEP9_SQUARE_UP_TO_FLOWER2:
                if (!follower.isBusy() || isStateTimedOut()) {
                    setShovelDown(true);
                    setState(State.STEP10_LOWER_SHOVEL);
                }
                break;

            case STEP10_LOWER_SHOVEL:
                if (stateTimer.seconds() >= SHOVEL_MOVE_SEC) {
                    startIntaking(pathIntoFlower2);
                    setState(State.STEP10_INTAKE_FLOWER2);
                }
                break;

            case STEP10_INTAKE_FLOWER2:
                if (nBallsCounted >= BALLS_PER_FLOWER || isStateTimedOut()) {
                    follower.setMaxPower(MAX_DRIVE_POWER);
                    driveTo(pathBackUpFromFlower2);
                    setState(State.STEP11_BACK_UP_FROM_FLOWER2);
                }
                break;

            case STEP11_BACK_UP_FROM_FLOWER2:
                if (!follower.isBusy() || isStateTimedOut()) {
                    setShovelDown(false);
                    setState(State.STEP11_RAISE_SHOVEL);
                }
                break;

            case STEP11_RAISE_SHOVEL:
                if (stateTimer.seconds() >= SHOVEL_MOVE_SEC) {
                    setIntake(false);
                    driveTo(pathReturnToShoot2);
                    setState(State.STEP12_RETURN_TO_SHOOT2);
                }
                break;

            case STEP12_RETURN_TO_SHOOT2:
                if (!follower.isBusy() || isStateTimedOut()) {
                    startVolley();
                    setState(State.STEP12_SHOOT_FLOWER2_POLLEN);
                }
                break;

            case STEP12_SHOOT_FLOWER2_POLLEN:
                if (updateVolley() || isStateTimedOut()) {
                    driveTo(pathToPark);
                    setState(State.STEP13_DRIVE_TO_PARK);
                }
                break;

            case STEP13_DRIVE_TO_PARK:
                if (!follower.isBusy() || isStateTimedOut()) {
                    shooter.setPower(0);
                    setIntake(false);
                    fireServo.setPosition(degToServoPos(FIRE_SERVO_HOME_DEG));
                    setState(State.DONE);
                }
                break;

            case DONE:
            default:

                // Nothing left to do. Pedro keeps holding the park spot while follower.update() runs.
                break;
        }
    }

    /*
     * WHAT: Switches to a new state: restarts the state timer and sets that state's timeout.
     * CALLED FROM: start() and runStateMachine().
     */
    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
        fStateTimeoutSec = timeoutFor(newState);
    }

    /*
     * WHAT: Returns how long (seconds) a state may take before the robot gives up on it and moves on.
     * CALLED FROM: setState().
     */
    private double timeoutFor(State forState) {
        switch (forState) {
            case STEP1_SHOOT_PRELOAD:
            case STEP8_SHOOT_FLOWER1_POLLEN:
            case STEP12_SHOOT_FLOWER2_POLLEN:
                return VOLLEY_TIMEOUT_SEC;
            case STEP2_BACK_UP_FROM_WALL:
            case STEP6_BACK_UP_FROM_FLOWER1:
            case STEP11_BACK_UP_FROM_FLOWER2:
                return SHORT_DRIVE_TIMEOUT_SEC;
            case STEP2_TURN_TO_FLOWER1:
            case STEP9_SQUARE_UP_TO_FLOWER2:
                return TURN_TIMEOUT_SEC;
            case STEP5_INTAKE_FLOWER1:
            case STEP10_INTAKE_FLOWER2:
                return INTAKE_TIMEOUT_SEC;
            case STEP7_DRIVE_TO_SHOOT2:
                return LONG_DRIVE_TIMEOUT_SEC;
            case STEP4_LOWER_SHOVEL:
            case STEP6_RAISE_SHOVEL:
            case STEP10_LOWER_SHOVEL:
            case STEP11_RAISE_SHOVEL:
                return SHOVEL_MOVE_SEC + 0.5;
            case DONE:
                return 1e9;   // never times out
            default:
                return DRIVE_TIMEOUT_SEC;
        }
    }

    /*
     * WHAT: Returns true if the current state has run longer than its timeout. It also remembers which
     *       state timed out, so telemetry can show it (useful for tuning).
     * CALLED FROM: runStateMachine().
     */
    private boolean isStateTimedOut() {
        if (stateTimer.seconds() < fStateTimeoutSec) return false;
        strLastTimeout = state.name() + String.format(" at %.1f s", stateTimer.seconds());
        return true;
    }

    // ============================ ROUTE ============================

    /*
     * WHAT: Copies this alliance's poses (RED or BLUE column) into the poses the steps use.
     * CALLED FROM: init().
     */
    private void pickAlliancePoses() {
        if (isBlueAlliance()) {
            startPose           = BLUE_START;
            clearWallPose       = BLUE_CLEAR_WALL;
            flower1ApproachPose = BLUE_FLOWER1_APPROACH;
            flower1IntakePose   = BLUE_FLOWER1_INTAKE;
            flower1BackupPose   = BLUE_FLOWER1_BACKUP;
            viaToShoot2Pose     = BLUE_VIA_TO_SHOOT2;
            shoot2Pose          = BLUE_SHOOT2;
            flower2ApproachPose = BLUE_FLOWER2_APPROACH;
            flower2IntakePose   = BLUE_FLOWER2_INTAKE;
            parkPose            = BLUE_PARK;
        } else {
            startPose           = RED_START;
            clearWallPose       = RED_CLEAR_WALL;
            flower1ApproachPose = RED_FLOWER1_APPROACH;
            flower1IntakePose   = RED_FLOWER1_INTAKE;
            flower1BackupPose   = RED_FLOWER1_BACKUP;
            viaToShoot2Pose     = RED_VIA_TO_SHOOT2;
            shoot2Pose          = RED_SHOOT2;
            flower2ApproachPose = RED_FLOWER2_APPROACH;
            flower2IntakePose   = RED_FLOWER2_INTAKE;
            parkPose            = RED_PARK;
        }
    }

    /*
     * WHAT: Builds every path from the poses. Each leg is a straight line (a BezierLine). Step 2's turn
     *       happens on the spot (turnTo), so the drive to flower 1 starts already facing the flower.
     * CALLED FROM: init().
     */
    private void buildPaths() {

        // Step 2: straight back off the wall, still facing the start heading.
        pathBackUpFromWall = leg(startPose, clearWallPose);

        // Step 3: from CLEAR_WALL (already turned to flower 1's heading) to FLOWER1_APPROACH.
        Pose clearWallTurned = new Pose(clearWallPose.getX(), clearWallPose.getY(), flower1ApproachPose.getHeading());
        pathToFlower1 = leg(clearWallTurned, flower1ApproachPose);

        // Steps 5 and 6: into flower 1, then back out to FLOWER1_BACKUP.
        pathIntoFlower1       = leg(flower1ApproachPose, flower1IntakePose);
        pathBackUpFromFlower1 = leg(flower1IntakePose, flower1BackupPose);

        // Step 7: two legs, FLOWER1_BACKUP -> VIA_TO_SHOOT2 -> SHOOT2, turning smoothly on each leg.
        pathToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(flower1BackupPose, viaToShoot2Pose))
                .setLinearHeadingInterpolation(flower1BackupPose.getHeading(), viaToShoot2Pose.getHeading())
                .addPath(new BezierLine(viaToShoot2Pose, shoot2Pose))
                .setLinearHeadingInterpolation(viaToShoot2Pose.getHeading(), shoot2Pose.getHeading())
                .build();

        // Steps 9 to 12: flower 2 and back.
        pathToFlower2         = leg(shoot2Pose, flower2ApproachPose);
        pathIntoFlower2       = leg(flower2ApproachPose, flower2IntakePose);
        pathBackUpFromFlower2 = leg(flower2IntakePose, flower2ApproachPose);
        pathReturnToShoot2    = leg(flower2ApproachPose, shoot2Pose);

        // Step 13: park.
        pathToPark = leg(shoot2Pose, parkPose);
    }

    /*
     * WHAT: Makes a one-leg straight path from one pose to another. Same heading at both ends = the
     *       robot keeps that heading; different headings = it turns smoothly from one to the other
     *       while it drives (Pedro's "linear heading interpolation", the short way round).
     * CALLED FROM: buildPaths().
     */
    private PathChain leg(Pose from, Pose to) {
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
     * WHAT: Starts following a path. true = hold the end position afterward (for shooting and intaking).
     * CALLED FROM: runStateMachine() and startIntaking().
     */
    private void driveTo(PathChain path) {
        follower.followPath(path, true);
    }

    // ============================ SHOOTING ============================

    /*
     * WHAT: Gets ready for a new 4-shot volley (shot count to 0, first shot waits for the RPM).
     * CALLED FROM: start() and runStateMachine() (before steps 8 and 12).
     */
    private void startVolley() {
        nShotsThisVolley = 0;
        shotPhase = ShotPhase.WAIT_FOR_RPM;
        shotTimer.reset();
        inBandTimer.reset();
    }

    /*
     * WHAT: Runs one loop of the current volley. Each shot: wait until SRM has been within
     *       FIRE_TOLERANCE_RPM of target for FIRE_SETTLE_SEC, swing the fire servo out, hold, bring it
     *       home, wait for it to get back, then the next shot waits for the RPM to recover.
     *       Returns true when all SHOTS_PER_VOLLEY shots are done.
     * CALLED FROM: runStateMachine(), in the three shooting states.
     */
    private boolean updateVolley() {
        if (nShotsThisVolley >= SHOTS_PER_VOLLEY) return true;
        switch (shotPhase) {
            case WAIT_FOR_RPM:
                boolean bIsSettled = bIsShooterInBand && inBandTimer.seconds() >= FIRE_SETTLE_SEC;
                boolean bIsWaitTooLong = shotTimer.seconds() >= SHOT_WAIT_TIMEOUT_SEC;
                if (bIsSettled || (bIsWaitTooLong && FIRE_ON_TIMEOUT)) {
                    fireServo.setPosition(degToServoPos(FIRE_SERVO_FIRE_DEG));
                    shotPhase = ShotPhase.FIRING;
                    shotTimer.reset();
                } else if (bIsWaitTooLong) {

                    // RPM never settled and FIRE_ON_TIMEOUT is false: skip this shot.
                    nShotsSkipped++;
                    nShotsThisVolley++;
                    shotTimer.reset();
                }
                break;
            case FIRING:
                if (shotTimer.seconds() >= FIRE_SERVO_HOLD_SEC) {
                    fireServo.setPosition(degToServoPos(FIRE_SERVO_HOME_DEG));
                    shotPhase = ShotPhase.RETURNING;
                    shotTimer.reset();
                }
                break;
            case RETURNING:
                if (shotTimer.seconds() >= FIRE_SERVO_RETURN_SEC) {
                    nShotsThisVolley++;
                    nShotsFiredTotal++;
                    shotPhase = ShotPhase.WAIT_FOR_RPM;
                    shotTimer.reset();

                    // Make the next shot wait for a fresh FIRE_SETTLE_SEC in band (RPM recovery).
                    inBandTimer.reset();
                }
                break;
        }
        return nShotsThisVolley >= SHOTS_PER_VOLLEY;
    }

    /*
     * WHAT: Reads SRM's speed and keeps a rolling average of the last RPM_AVG_SAMPLES readings. Then
     *       checks whether that average is within FIRE_TOLERANCE_RPM of target; the in-band timer
     *       restarts whenever it is not.
     * CALLED FROM: loop(), every loop.
     */
    private void readShooterRpm() {
        fRpmNow = Math.abs(shooter.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV);
        rpmSamples[(int) nRpmSampleIndex] = fRpmNow;
        nRpmSampleIndex = (nRpmSampleIndex + 1) % RPM_AVG_SAMPLES;
        if (nRpmSampleCount < RPM_AVG_SAMPLES) nRpmSampleCount++;
        double fSum = 0;
        for (long i = 0; i < nRpmSampleCount; i++) {
            fSum += rpmSamples[(int) i];
        }
        fRpmAvg = fSum / nRpmSampleCount;

        // A full set of readings is needed, so one lucky reading can't pass the gate.
        bIsShooterInBand = nRpmSampleCount == RPM_AVG_SAMPLES
                && Math.abs(fRpmAvg - POLLEN_TARGET_RPM) <= FIRE_TOLERANCE_RPM;
        if (!bIsShooterInBand) inBandTimer.reset();
    }

    /*
     * WHAT: Converts RPM (turns per minute) to encoder ticks per second, the unit setVelocity() wants.
     * CALLED FROM: start().
     */
    private double rpmToTicksPerSec(double fRpm) {
        return fRpm * SHOOTER_TICKS_PER_REV / 60.0;
    }

    // ============================ SHOVEL, INTAKE, BALL COUNTER ============================

    /*
     * WHAT: Starts an intake step: ball count to 0, intake ON, slow speed, drive into the flower.
     * CALLED FROM: runStateMachine() (steps 5 and 10).
     */
    private void startIntaking(PathChain pathIntoFlower) {
        nBallsCounted = 0;
        setIntake(true);
        follower.setMaxPower(INTAKE_DRIVE_POWER);
        driveTo(pathIntoFlower);
    }

    /*
     * WHAT: Moves the shovel (servo_FJ and FJF2 together) DOWN to scoop, or UP.
     * CALLED FROM: init() and runStateMachine().
     */
    private void setShovelDown(boolean bIsDown) {
        bIsShovelDown = bIsDown;
        if (bIsDown) {
            shovelServo4.setPosition(degToServoPos(fServo4DownDeg));
            shovelServo5.setPosition(degToServoPos(fServo5DownDeg));
        } else {
            shovelServo4.setPosition(degToServoPos(SERVO4_UP_DEG));
            shovelServo5.setPosition(degToServoPos(fServo5UpDeg));
        }
    }

    /*
     * WHAT: Turns the intake ON (pulling balls in at INTAKE_POWER) or OFF.
     * CALLED FROM: runStateMachine(), startIntaking() and stop().
     */
    private void setIntake(boolean bIsOn) {
        bIsIntakeOn = bIsOn;
        double fPower = 0;
        if (bIsOn) fPower = INTAKE_POWER;
        intakeLeft.setPower(fPower);
        intakeRight.setPower(fPower);
    }

    /*
     * WHAT: Loads the shovel angles the TeleOp saved, so autonomous uses the same tuned angles.
     * CALLED FROM: init().
     */
    private void loadShovelAngles() {
        fServo4DownDeg = readSavedAngle(SERVO4_DOWN_FILE, SERVO4_DOWN_DEG);
        fServo5UpDeg   = readSavedAngle(SERVO5_UP_FILE, SERVO5_UP_DEG);
        fServo5DownDeg = readSavedAngle(SERVO5_DOWN_FILE, SERVO5_DOWN_DEG);
    }

    /*
     * WHAT: Reads one angle (degrees) from a file in /sdcard/FIRST/settings/. Returns fDefaultDeg if
     *       the file is missing, unreadable, or outside 0-180 degrees.
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

    /*
     * WHAT: Finds the ball-counting limit switches. tryGet returns null (instead of stopping the op
     *       mode) for a switch that is not in the robot config; missing switches are just skipped.
     * CALLED FROM: init().
     */
    private void setupBallSwitches() {
        ballSwitches = new DigitalChannel[BALL_SWITCH_NAMES.length];
        bIsSwitchPressed = new boolean[BALL_SWITCH_NAMES.length];
        switchDebounce = new ElapsedTime[BALL_SWITCH_NAMES.length];
        for (int i = 0; i < BALL_SWITCH_NAMES.length; i++) {
            ballSwitches[i] = hardwareMap.tryGet(DigitalChannel.class, BALL_SWITCH_NAMES[i]);
            if (ballSwitches[i] != null) ballSwitches[i].setMode(DigitalChannel.Mode.INPUT);
            switchDebounce[i] = new ElapsedTime();
        }
    }

    /*
     * WHAT: Adds 1 to nBallsCounted each time either limit switch is newly pressed. The debounce timer
     *       ignores the few milliseconds of contact bounce, so one ball never counts as 2 or 3.
     * CALLED FROM: loop(), every loop.
     */
    private void countBalls() {
        for (int i = 0; i < ballSwitches.length; i++) {
            if (ballSwitches[i] == null) continue;
            boolean bIsPressed = ballSwitches[i].getState() != SWITCH_PRESSED_IS_LOW;
            if (bIsPressed != bIsSwitchPressed[i] && switchDebounce[i].seconds() >= SWITCH_DEBOUNCE_SEC) {
                if (bIsPressed) nBallsCounted++;
                bIsSwitchPressed[i] = bIsPressed;
                switchDebounce[i].reset();
            }
        }
    }

    /*
     * WHAT: Returns a short line saying how many ball switches were found (for telemetry).
     * CALLED FROM: init_loop() and showTelemetry().
     */
    private String ballCounterStatus() {
        long nFound = 0;
        for (DigitalChannel ballSwitch : ballSwitches) {
            if (ballSwitch != null) nFound++;
        }
        if (nFound == 0) return "OFF (no limit1/limit2 in config): intake steps use the 1.5 s timeout";
        return nFound + " switch(es) found";
    }

    // ============================ HELPERS ============================

    /*
     * WHAT: Converts a servo angle in degrees to the 0-1 position the servo wants (degrees / 180).
     *       Clamping keeps it inside 0-1 (if it would go past a limit, it stops AT the limit).
     * CALLED FROM: everywhere a servo is moved.
     */
    private double degToServoPos(double fDeg) {
        return Math.max(0.0, Math.min(1.0, fDeg / SERVO_RANGE_DEG));
    }

    /*
     * WHAT: Shows the current state, how long it has run, the shooter RPM, the pose, the shots and
     *       balls, the shovel and intake, and the last state that timed out.
     * CALLED FROM: loop(), every loop.
     */
    private void showTelemetry() {
        Pose pose = follower.getPose();
        telemetry.addData("State", "%s  (%.1f s)", state.name(), stateTimer.seconds());
        telemetry.addData("SRM RPM", "%.0f avg  %.0f raw  / target %.0f", fRpmAvg, fRpmNow, POLLEN_TARGET_RPM);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.getX(), pose.getY(),
                Math.toDegrees(pose.getHeading()));
        telemetry.addData("Shots", "%d fired, %d skipped (this volley %d/%d)",
                nShotsFiredTotal, nShotsSkipped, nShotsThisVolley, SHOTS_PER_VOLLEY);
        telemetry.addData("Balls counted", "%d  (%s)", nBallsCounted, ballCounterStatus());
        String strShovel = "UP";
        if (bIsShovelDown) strShovel = "DOWN";
        String strIntake = "off";
        if (bIsIntakeOn) strIntake = "ON";
        telemetry.addData("Shovel / intake", strShovel + " / " + strIntake);
        telemetry.addData("Last timeout", strLastTimeout);
        telemetry.update();
    }
}

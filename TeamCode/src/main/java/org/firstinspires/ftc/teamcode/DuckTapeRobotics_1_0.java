package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

/*
 * Mecanum drive + 2 velocity-controlled shooters + 2-motor intake + 4 servos, all mechanisms on toggles.
 *
 * GAMEPAD 1 — DRIVER (driving)
 *   Left stick   : drive. FIELD-CENTRIC: stick forward = away from the driver, no matter which
 *                  way the robot faces. Set FIELD_CENTRIC = false below to go back to robot-centric.
 *   Right stick X: turn
 *   Start        : reset heading (point robot away from driver, then press)
 *   X            : toggle SNAP-TO-ANGLE (auto-turns to SNAP_HEADING_DEG and holds it
 *                  while you strafe; moving the right stick cancels it)
 *   Y            : SERVO4 toggle: starts UP, press to swing DOWN 88 degrees, press again to go back UP
 *   Left bumper  : hold for slow drive (precision)
 *   D-pad left  : robot pulls right? tap to slow the left side
 *   D-pad right  : robot pulls left?  tap to slow the right side
 *   Right bumper : toggle ALL lights ON/OFF (Prism strips + rgblight; they start ON)
 *
 * GAMEPAD 2 — OPERATOR (intake + shooters + servos)
 *   Right trigger: toggle intake FORWARD (pull in) ON/OFF
 *   Left trigger : toggle intake REVERSE (spit out) ON/OFF
 *   D-pad left   : toggle LEFT shooter ON/OFF
 *   D-pad right  : toggle RIGHT shooter ON/OFF
 *   D-pad up/down: LEFT shooter speed  +100 / -100 RPM per tap
 *   B / X        : RIGHT shooter speed +100 / -100 RPM per tap (B = up, X = down)
 *   Right bumper : SERVO1 opens, waits SERVO1_HOLD_SEC (0.45s), then returns home
 *   Left bumper  : SERVO2 goes to 90 degrees, waits SERVO2_HOLD_SEC (0.3s), then returns home
 *   A            : SERVO3 goes up 45 degrees, waits SERVO3_HOLD_SEC (0.55s), then returns home
 *   Y            : SHAKE: robot strafes left/right quickly for SHAKE_TIME_SEC to unstick balls
 *   Right stick button (press the stick down): first Prism strip color:
 *                  RED -> GREEN -> BLUE -> WHITE -> back to RED
 *   Rumble/LED   : flywheel "ready to fire" feedback
 *
 * HARDWARE CONFIG NAMES (must match the Driver Station config exactly)
 *   FLM (front left motor), FRM (front right motor),
 *   BLM (back left motor),  BRM (back right motor),
 *   SLM (shooter left motor), SRM (shooter right motor),
 *   ILM (intake left motor),  IRM (intake right motor)
 *   servo1, servo2, servo3, servo4 (servo ports, configured as "Servo")
 *   imu (the Control Hub's built-in IMU, I2C bus 0; usually already in the config)
 *   prism (goBILDA Prism RGB LED Driver, Control Hub I2C bus 1)
 *   rgblight (goBILDA RGB Indicator Light, servo port, configured as "Servo")
 *     RED = shooters off, YELLOW = spinning up, GREEN = ready to fire
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

    // ---- Shooter PIDF (from the Shooter PIDF Tuner OpMode) ----
    // false = use the SDK's built-in defaults. Set true after tuning.
    private static final boolean USE_CUSTOM_PIDF = false;
    private static final double SHOOTER_P = 1.8408;
    private static final double SHOOTER_I = 0.1841;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F_LEFT  = 14.0030;   // each shooter has its own F
    private static final double SHOOTER_F_RIGHT = 18.4084;

    // Shooter speed. Each shooter has its own target.
    //   LEFT  shooter: gamepad2 D-pad up = +RPM_STEP, D-pad down = -RPM_STEP
    //   RIGHT shooter: gamepad2 B        = +RPM_STEP, X          = -RPM_STEP
    private static final double START_RPM_LEFT  = 2400;  // left shooter speed at the start of the match
    private static final double START_RPM_RIGHT = 2400;  // right shooter speed at the start of the match
    private static final double RPM_STEP        = 100;   // change per D-pad tap
    private static final double MIN_RPM         = 1000;  // D-pad down stops here
    private static final double READY_TOLERANCE = 50;    // "ready to fire" if within this many RPM

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
    private static final double  SERVO2_HOME_DEG   = 20;   // kept off 0 so the servo isn't jammed at its end stop
    private static final double  SERVO2_TARGET_DEG = 110;  // still a 90-degree swing
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
    private static final double  SERVO3_HOLD_SEC   = 0.55;  // how long it stays up before returning

    // ---- Servo4 (gamepad1 Y) ----
    // TOGGLE: starts UP at SERVO4_HOME_DEG. Press Y to swing DOWN 88 degrees, press Y again to go back up.
    // No timer; it stays where you put it. Uses the same SERVO_RANGE_DEG as the others.
    // If it buzzes at the bottom (end stop), lower SERVO4_SWING_DEG (e.g. 70).
    private static final double  SERVO4_HOME_DEG   = 0;    // start position (up) - at the end of travel; raise to 5-10 if it buzzes at rest
    private static final double  SERVO4_SWING_DEG  = 88;   // how far DOWN it goes when toggled (0 + 88 = 88), fixed
    private static final double  SERVO4_OPEN_DEG   = SERVO4_HOME_DEG + SERVO4_SWING_DEG;  // 88 deg = down
    private static final boolean SERVO4_REVERSED   = true;   // reversed: down swing now goes the other way

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
    private static final boolean FIELD_CENTRIC = true;

    // ---- Heading hold (fixes curving) ----
    // When you drive without touching the turn stick, the IMU keeps the robot
    // pointed the same way, correcting for a weak/strong wheel or uneven drag.
    private static final boolean HEADING_HOLD   = false;
    private static final double  HOLD_KP        = 1.5;   // raise if it still curves, lower if it wiggles
    private static final double  HOLD_MAX_POWER = 0.3;   // cap so correction never overpowers the driver

    // ---- Drive balance (D-pad LEFT / RIGHT during TeleOp) ----
    // Robot pulls RIGHT  -> tap D-pad LEFT  (slows the left side)
    // Robot pulls LEFT   -> tap D-pad RIGHT (slows the right side)
    // Each tap = 0.05. Telemetry shows the value; put it here to make it permanent.
    private static final double DRIVE_BALANCE_START = 0.0;
    private static final double DRIVE_BALANCE_STEP  = 0.05;
    private static final double DRIVE_BALANCE_MAX   = 0.5;

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

    // ---- Prism LED lights (gamepad1 Right bumper) ----
    // The 4 strips are daisy-chained, so the Prism sees ONE long string of LEDs.
    // List each strip's LED count IN THE ORDER THEY ARE CHAINED, starting from the
    // strip plugged into the Prism. Kit strips: short = 6 LEDs, long = 12 LEDs.
    // If the wrong strip gets the wrong pattern, reorder these numbers.
    private static final int[] STRIP_LEDS      = {12, 12, 6, 6};
    private static final int   LED_BRIGHTNESS  = 60;     // 0-100
    private static final boolean LIGHTS_START_ON = true;

    // First strip in the chain cycles through these on gamepad2 right stick button. Starts on the first one.
    private static final Color[]  STRIP1_COLORS      = {Color.RED, Color.GREEN, Color.BLUE, Color.WHITE};
    private static final String[] STRIP1_COLOR_NAMES = {"RED", "GREEN", "BLUE", "WHITE"};

    // ---- goBILDA RGB Indicator Light (servo port) ----
    // setPosition() picks the color. Do NOT reverse this servo or colors come out wrong.
    private static final double RGB_OFF    = 0.000;
    private static final double RGB_RED    = 0.277;
    private static final double RGB_YELLOW = 0.388;
    private static final double RGB_GREEN  = 0.500;

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

    // One animation per strip
    private final PrismAnimations.Solid         stripSolid  = new PrismAnimations.Solid(Color.RED);
    private final PrismAnimations.PoliceLights  stripSiren  = new PrismAnimations.PoliceLights();
    private final PrismAnimations.RainbowSnakes stripRainbow = new PrismAnimations.RainbowSnakes();
    private final PrismAnimations.DroidScan     stripScanner = new PrismAnimations.DroidScan(Color.RED);
    private boolean lightsOn = false;
    private boolean lastRB1;
    private int strip1ColorIndex = 0;   // 0 = RED
    private boolean lastRT2, lastLT2, lastRSB2;
    private static final double TRIGGER_PRESSED = 0.5;  // trigger counts as "pressed" past halfway

    // ---- State ----
    private double targetRpmLeft  = START_RPM_LEFT;   // current left shooter target
    private double targetRpmRight = START_RPM_RIGHT;  // current right shooter target
    private boolean intakeOn = false;
    private boolean intakeReverse = false;
    private boolean shooterLeftOn  = false;
    private boolean shooterRightOn = false;
    private boolean servo1Out = false;  // false = home, true = at SERVO1_TARGET_DEG
    private final ElapsedTime servo1Timer = new ElapsedTime();  // time since servo1 was sent out
    private boolean servo3Up = false;   // false = home, true = up 45 degrees
    private final ElapsedTime servo3Timer = new ElapsedTime();
    private boolean servo2Out = false;
    private final ElapsedTime servo2Timer = new ElapsedTime();
    private boolean servo4Open = false; // false = up (home), true = down
    private boolean shaking = false;    // shake in progress
    private final ElapsedTime shakeTimer = new ElapsedTime();
    private boolean lastY2;
    private boolean wasReady = false;   // used to rumble once when flywheels reach speed

    // Previous button states (for detecting a single press)
    private boolean lastUp, lastDown, lastX, lastRB, lastLB, lastA2, lastB2, lastX2, lastY1, lastLeft2, lastRight2;

    private boolean snapping = false;      // heading lock active
    private boolean snapOnTarget = false;  // within tolerance of snap heading
    private boolean holding = false;       // heading hold active
    private double  holdHeading = 0;       // heading (radians) being held

    // Live drive balance (D-pad left/right). Starts at DRIVE_BALANCE_START.
    // Once you find a value that drives straight, copy it into DRIVE_BALANCE_START.
    private double driveBalance = DRIVE_BALANCE_START;
    private boolean lastLeft, lastRight;

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
        servo1       = hardwareMap.get(Servo.class, "servo1");
        servo2       = hardwareMap.get(Servo.class, "servo2");
        servo3       = hardwareMap.get(Servo.class, "servo3");
        servo4       = hardwareMap.get(Servo.class, "servo4");
        rgbLight     = hardwareMap.get(ServoImplEx.class, "rgblight");
        rgbLight.setPwmRange(new PwmControl.PwmRange(500, 2500)); // goBILDA color chart uses 500-2500us
        rgbLight.setPwmEnable();
        rgbLight.setPosition(RGB_RED);   // red at init: shooters off

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
        if (USE_CUSTOM_PIDF) {
            shooterLeft.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F_LEFT);
            shooterRight.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F_RIGHT);
        }

        telemetry.addLine("GP1: drive, X=snap  Y=servo4 open/close  LB=slow  RB=lights  Start=reset heading");
        telemetry.addLine("GP2: Dpad left=LEFT shooter on/off  Dpad right=RIGHT shooter on/off");
        telemetry.addLine("GP2: Dpad up/down=LEFT RPM +/-100  B/X=RIGHT RPM +/-100");
        telemetry.addLine("GP2: RB=servo1  LB=servo2  A=servo3  Y=shake");
        telemetry.addLine("GP2: RT=intake  LT=intake reverse  R-stick click=strip1 color");
        telemetry.addLine("Robot should face AWAY from driver at init.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // MANUAL bulk caching: clear once at the top of every loop,
            // or every encoder/velocity read will return stale data.
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            drive();
            handleToggles();
            runMechanisms();
            readyFeedback();
            updateTelemetry();
        }

        stopAll();
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

        // ---- Shake (gamepad2 Y) ----
        // Overrides driving: pure strafe that flips direction every SHAKE_FLIP_SEC.
        boolean y2 = gamepad2.y;
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
        // Gamepad 2 (operator): intake on the triggers (past halfway counts as a press)
        boolean rt2 = gamepad2.right_trigger > TRIGGER_PRESSED;
        boolean lt2 = gamepad2.left_trigger  > TRIGGER_PRESSED;
        // Gamepad 2 (operator): shooters + servos
        boolean up = gamepad2.dpad_up, down = gamepad2.dpad_down;
        boolean rb = gamepad2.right_bumper;
        boolean lb = gamepad2.left_bumper;
        boolean a2 = gamepad2.a;
        boolean b2 = gamepad2.b;
        boolean x2 = gamepad2.x;
        boolean y1 = gamepad1.y;   // servo4 toggle (driver)
        boolean left2 = gamepad2.dpad_left, right2 = gamepad2.dpad_right;

        // Intake: RT toggles forward, LT toggles reverse. Pressing one cancels the other.
        if (rt2 && !lastRT2) {
            intakeOn = !intakeOn;
            if (intakeOn) intakeReverse = false;
        }
        if (lt2 && !lastLT2) {
            intakeReverse = !intakeReverse;
            if (intakeReverse) intakeOn = false;
        }
        // D-pad left / right: each shooter on its own.
        if (left2 && !lastLeft2) {
            shooterLeftOn = !shooterLeftOn;
        }
        if (right2 && !lastRight2) {
            shooterRightOn = !shooterRightOn;
        }
        // Each servo: one press starts its cycle. Presses during the cycle are ignored.
        if (rb && !lastRB && !servo1Out) {
            servo1Out = true;
            servo1Timer.reset();
        }
        if (a2 && !lastA2 && !servo3Up) {
            servo3Up = true;
            servo3Timer.reset();
        }
        if (lb && !lastLB && !servo2Out) {
            servo2Out = true;
            servo2Timer.reset();
        }
        // Servo4: gamepad1 Y flips between open and closed.
        if (y1 && !lastY1) {
            servo4Open = !servo4Open;
        }
        // LEFT shooter speed: D-pad up = +RPM_STEP, D-pad down = -RPM_STEP.
        // Stops at MIN_RPM / SHOOTER_MAX_RPM.
        if (up && !lastUp) {
            targetRpmLeft = Math.min(targetRpmLeft + RPM_STEP, SHOOTER_MAX_RPM);
        }
        if (down && !lastDown) {
            targetRpmLeft = Math.max(targetRpmLeft - RPM_STEP, MIN_RPM);
        }
        // RIGHT shooter speed: B = +RPM_STEP, X = -RPM_STEP.
        if (b2 && !lastB2) {
            targetRpmRight = Math.min(targetRpmRight + RPM_STEP, SHOOTER_MAX_RPM);
        }
        if (x2 && !lastX2) {
            targetRpmRight = Math.max(targetRpmRight - RPM_STEP, MIN_RPM);
        }

        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        if (left && !lastLeft) {
            driveBalance = Math.min(driveBalance + DRIVE_BALANCE_STEP, DRIVE_BALANCE_MAX);
        }
        if (right && !lastRight) {
            driveBalance = Math.max(driveBalance - DRIVE_BALANCE_STEP, -DRIVE_BALANCE_MAX);
        }
        lastLeft = left; lastRight = right;

        // Prism lights: gamepad1 right bumper flips them on/off.
        // Only talks to the Prism on the press, so it doesn't slow the loop.
        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !lastRB1) {
            setLights(!lightsOn);
        }
        lastRB1 = rb1;

        // First Prism strip: gamepad2 right stick button steps to the next color and loops back to red.
        boolean rsb2 = gamepad2.right_stick_button;
        if (rsb2 && !lastRSB2) {
            strip1ColorIndex = (strip1ColorIndex + 1) % STRIP1_COLORS.length;
            stripSolid.setPrimaryColor(STRIP1_COLORS[strip1ColorIndex]);
            // Only send it if the lights are on; if they're off, the new color shows when RB turns them back on.
            if (lightsOn) {
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, stripSolid);
            }
        }
        lastRSB2 = rsb2;

        lastRT2 = rt2; lastLT2 = lt2; lastUp = up; lastDown = down; lastRB = rb; lastLB = lb; lastA2 = a2;
        lastB2 = b2; lastX2 = x2; lastY1 = y1; lastLeft2 = left2; lastRight2 = right2;
    }

    // ---------------- Apply mechanism outputs ----------------
    private void runMechanisms() {
        double intakePower = intakeOn ? INTAKE_POWER : intakeReverse ? -INTAKE_POWER : 0;
        intakeLeft.setPower(intakePower);
        intakeRight.setPower(intakePower);

        // Auto-return: timer-based, so driving and shooters keep working during the wait.
        if (servo1Out && servo1Timer.seconds() >= SERVO1_HOLD_SEC) {
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

        servo4.setPosition(degToServoPos(servo4Open ? SERVO4_OPEN_DEG : SERVO4_HOME_DEG));

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
        PrismAnimations.AnimationBase[] anims = {stripSolid, stripSiren, stripRainbow, stripScanner};

        // Strip 1: solid color, starts RED (gamepad2 right stick button cycles it)
        stripSolid.setPrimaryColor(STRIP1_COLORS[0]);

        // Strip 2: siren (red / white / blue police lights)
        stripSiren.setPoliceLightsStyle(PrismAnimations.PoliceLights.PoliceLightsStyle.Style1);
        stripSiren.setPeriod(500);          // ms per loop; lower = faster flashing

        // Strip 3: rainbow snakes chasing down the strip
        stripRainbow.setNumberOfSnakes(2);
        stripRainbow.setSnakeLength(3);
        stripRainbow.setSpacingBetween(3);
        stripRainbow.setSpeed(0.5f);

        // Strip 4: "Knight Rider" scanner bouncing back and forth
        stripScanner.setEyeWidth(1);
        stripScanner.setTrailWidth(2);
        stripScanner.setSpeed(0.5f);

        int start = 0;
        for (int i = 0; i < anims.length; i++) {
            int stop = start + STRIP_LEDS[i] - 1;   // stop index is the LAST LED, inclusive
            anims[i].setStartIndex(start);
            anims[i].setStopIndex(stop);
            anims[i].setBrightness(LED_BRIGHTNESS);
            start = stop + 1;
        }
    }

    private void setLights(boolean on) {
        if (on) {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, stripSolid);
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_1, stripSiren);
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, stripRainbow);
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_3, stripScanner);
            rgbLight.setPosition(RGB_RED);   // readyFeedback() takes over from here
        } else {
            prism.clearAllAnimations();
            rgbLight.setPosition(RGB_OFF);
        }
        lightsOn = on;
    }

    // ---------------- Helpers ----------------

    // Converts degrees to the 0.0-1.0 servo position, clamped to the servo's range.
    private double degToServoPos(double deg) {
        return Math.max(0.0, Math.min(1.0, deg / SERVO_RANGE_DEG));
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
        double l = Math.abs(ticksPerSecToRpm(shooterLeft.getVelocity()));
        double r = Math.abs(ticksPerSecToRpm(shooterRight.getVelocity()));
        // Ready when every shooter that is ON is at its target. Shooters that are OFF are ignored.
        boolean leftOk  = !shooterLeftOn  || Math.abs(l - targetRpmLeft)  < READY_TOLERANCE;
        boolean rightOk = !shooterRightOn || Math.abs(r - targetRpmRight) < READY_TOLERANCE;
        return leftOk && rightOk;
    }

    // ---------------- Driver feedback when flywheels hit target speed ----------------
    // Rumble: one double-buzz the moment every running shooter reaches its target RPM.
    // LED (PS4/PS5 controllers only): RED = off, YELLOW = spinning up, GREEN = ready.
    // Logitech F310 controllers have no rumble or LED; use telemetry instead.
    private void readyFeedback() {
        boolean ready = shootersReady();

        if (ready && !wasReady) {
            gamepad2.rumbleBlips(2);
        }
        wasReady = ready;

        // Gamepad LED and the robot's RGB light show the same thing.
        if (!shooterLeftOn && !shooterRightOn) {
            gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (ready) {
            gamepad2.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // Robot RGB light: same colors, but only while the lights are toggled ON (GP1 RB).
        if (!lightsOn) {
            rgbLight.setPosition(RGB_OFF);
        } else if (!shooterLeftOn && !shooterRightOn) {
            rgbLight.setPosition(RGB_RED);
        } else if (ready) {
            rgbLight.setPosition(RGB_GREEN);
        } else {
            rgbLight.setPosition(RGB_YELLOW);
        }
    }

    private long lastLoopNs = System.nanoTime();
    private double minVolts = 99;

    private double batteryVoltage() {
        double v = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double reading = s.getVoltage();
            if (reading > 0) v = Math.min(v, reading);
        }
        return v == Double.POSITIVE_INFINITY ? 0 : v;
    }

    private void updateTelemetry() {
        long now = System.nanoTime();
        telemetry.addData("Loop time (ms)", "%.1f", (now - lastLoopNs) / 1e6);
        lastLoopNs = now;
        // ---- Battery (lowest hub reading; watch for sag while driving) ----
        double volts = batteryVoltage();
        if (volts > 0) minVolts = Math.min(minVolts, volts);
        telemetry.addData("Battery", "%.2f V   (lowest this run: %.2f V)", volts, minVolts);

        // ---- Drive diagnostics (remove once driving is sorted) ----
        telemetry.addData("Stick LX / LY / RX", "%.2f / %.2f / %.2f",
                gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.addData("Power FL / FR", "%.2f / %.2f", frontLeft.getPower(), frontRight.getPower());
        telemetry.addData("Power BL / BR", "%.2f / %.2f", backLeft.getPower(), backRight.getPower());

        telemetry.addData("Heading (deg)", "%.1f",
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Drive balance", "%+.2f  (%s)", driveBalance,
                driveBalance > 0 ? "left side slowed" : driveBalance < 0 ? "right side slowed" : "even");
        telemetry.addData("Heading hold", holding
                ? String.format("ON @ %.1f°", Math.toDegrees(holdHeading)) : "off");
        telemetry.addData("Snap", !snapping ? "off"
                : snapOnTarget ? "LOCKED @ " + SNAP_HEADING_DEG + "°"
                : "turning to " + SNAP_HEADING_DEG + "°");
        telemetry.addData("Shake", shaking ? "SHAKING" : "off  (GP2 Y)");
        telemetry.addData("Lights", lightsOn ? "ON  (GP1 RB)" : "OFF  (GP1 RB)");
        telemetry.addData("Strip 1 color", "%s  (GP2 right stick click)", STRIP1_COLOR_NAMES[strip1ColorIndex]);
        telemetry.addData("Intake", "%s  (GP2 RT / LT)", intakeOn ? "ON" : intakeReverse ? "REVERSE" : "OFF");
        telemetry.addData("Servo1", "%s  (pos %.2f)",
                servo1Out ? String.format("OUT, returning in %.1fs",
                        Math.max(0, SERVO1_HOLD_SEC - servo1Timer.seconds())) : "HOME",
                servo1.getPosition());
        telemetry.addData("Servo2", "%s  (pos %.2f)",
                servo2Out ? String.format("OUT, returning in %.1fs",
                        Math.max(0, SERVO2_HOLD_SEC - servo2Timer.seconds())) : "HOME",
                servo2.getPosition());
        telemetry.addData("Servo3", "%s  (pos %.2f)",
                servo3Up ? String.format("UP, returning in %.1fs",
                        Math.max(0, SERVO3_HOLD_SEC - servo3Timer.seconds())) : "HOME",
                servo3.getPosition());
        telemetry.addData("Servo4", "%s  (pos %.2f)",
                servo4Open ? "DOWN" : "UP", servo4.getPosition());
        telemetry.addData("Shooters", "LEFT %s  /  RIGHT %s",
                shooterLeftOn ? "ON" : "OFF", shooterRightOn ? "ON" : "OFF");
        telemetry.addData("Left RPM",  "%.0f / target %.0f  (D-pad up/down)",
                Math.abs(ticksPerSecToRpm(shooterLeft.getVelocity())), targetRpmLeft);
        telemetry.addData("Right RPM", "%.0f / target %.0f  (B up / X down)",
                Math.abs(ticksPerSecToRpm(shooterRight.getVelocity())), targetRpmRight);
        telemetry.addData("READY TO FIRE", shootersReady() ? ">>> YES <<<" : "no");
        telemetry.update();
    }

    private void stopAll() {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight, intakeLeft, intakeRight}) {
            m.setPower(0);
        }
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        prism.clearAllAnimations();   // lights off when the OpMode stops
        rgbLight.setPosition(RGB_OFF);
    }
}
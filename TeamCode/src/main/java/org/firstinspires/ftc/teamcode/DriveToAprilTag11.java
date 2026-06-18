package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * DriveToAprilTag11
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   A standalone LinearOpMode that finds AprilTag 11 and drives up to it using raw
 *   motor control (no Pedro Pathing). It runs three sequential phases:
 *     1. Search  — spins slowly until the tag appears (3-second timeout).
 *     2. Align   — turns in place until tx (horizontal offset) is within ±2°.
 *     3. Approach — drives straight until the tag's target area reaches 38%.
 *   Stops and exits once arrived (or if the tag is never found).
 *
 * WHAT IT'S FOR:
 *   A simple, self-contained AprilTag approach routine that does not rely on Pedro
 *   Pathing. Useful as a reference or a fallback when the Pedro follower isn't needed.
 *   For the combined vision + triangle path, see SampleAutoTriangleAprilTag.
 *
 * HARDWARE REQUIRED (names must match the Driver Hub robot config):
 *   - "front_left_motor", "front_right_motor", "back_left_motor", "back_right_motor"
 *   - "limelight" (Limelight 3A camera, AprilTag pipeline 0)
 * =====================================================================================
 */

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Autonomous(name = "Drive To AprilTag 11", group = "Autonomous")
public class DriveToAprilTag11 extends LinearOpMode {

    // --- Hardware ---
    private Limelight3A limelight;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // --- Tuning Constants ---
    private static final int    TARGET_TAG_ID     = 11;

    // How centered the tag needs to be before we drive (degrees)
    private static final double TX_DEADBAND        = 2.0;

    // Stop driving forward when target area hits this % of the image.
    private static final double TARGET_AREA_CLOSE  = 38.0;

    // Turn speed when aligning
    private static final double TURN_POWER         = 0.25;

    // Drive speed when approaching
    private static final double DRIVE_POWER        = 0.4;

    // Proportional gain for turning
    private static final double KP_TURN            = 0.02;

    // How long to search before giving up (seconds)
    private static final double SEARCH_TIMEOUT_S   = 3.0;

    private static final int    PIPELINE_INDEX     = 0;

    @Override
    public void runOpMode() {
        TelemetryLog log = new TelemetryLog(PanelsTelemetry.INSTANCE.getTelemetry(), "DriveToAprilTag11");

        // --- Init Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.setPollRateHz(100);
        limelight.start();

        // --- Init Motors ---
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        log.debug("Status", "Ready. Waiting for start...");
        log.update(telemetry);

        waitForStart();

        // -------------------------------------------------------
        // PHASE 1: Search for AprilTag 11
        // -------------------------------------------------------
        log.debug("Phase", "Searching for tag " + TARGET_TAG_ID + "...");
        log.update(telemetry);

        boolean tagFound = false;
        double searchStart = getRuntime();

        while (opModeIsActive() && !tagFound && (getRuntime() - searchStart) < SEARCH_TIMEOUT_S) {
            LLResultTypes.FiducialResult tag = getTag11();
            if (tag != null) {
                tagFound = true;
            } else {
                log.debug("Phase", String.format("Searching... (%.1fs)", getRuntime() - searchStart));
                log.update(telemetry);
                sleep(50);
            }
        }

        if (!tagFound) {
            log.debug("Phase", "Tag 11 not found. Stopping.");
            log.update(telemetry);
            log.save();
            limelight.stop();
            return;
        }

        // -------------------------------------------------------
        // PHASE 2: Turn to face the tag (center tx on 0)
        // -------------------------------------------------------
        log.debug("Phase", "Aligning...");
        log.update(telemetry);

        while (opModeIsActive()) {
            LLResultTypes.FiducialResult tag = getTag11();
            if (tag == null) continue;

            double tx = tag.getTargetXDegrees();

            if (Math.abs(tx) <= TX_DEADBAND) {
                stopDrive();
                break;
            }

            // Proportional turn: further off-center = faster turn
            double turnSpeed = Math.max(TURN_POWER, Math.abs(tx * KP_TURN));
            turnSpeed = Math.min(turnSpeed, TURN_POWER); // cap at TURN_POWER

            if (tx > 0) {
                // Tag is to the right â€” turn right
                setMotors(turnSpeed, -turnSpeed, turnSpeed, -turnSpeed);
            } else {
                // Tag is to the left â€” turn left
                setMotors(-turnSpeed, turnSpeed, -turnSpeed, turnSpeed);
            }

            log.debug("Phase", "Aligning");
            log.addData("tx (horizontal offset)", tx);
            log.update(telemetry);
        }

        // -------------------------------------------------------
        // PHASE 3: Drive forward until very close (ta threshold)
        // -------------------------------------------------------
        log.debug("Phase", "Approaching...");
        log.update(telemetry);

        while (opModeIsActive()) {
            LLResultTypes.FiducialResult tag = getTag11();
            if (tag == null) continue;

            double ta = tag.getTargetArea();
            double tx = tag.getTargetXDegrees();

            if (ta >= TARGET_AREA_CLOSE) {
                stopDrive();
                break;
            }

            // Small correction while driving to stay centered
            double correction = tx * KP_TURN;
            setMotors(
                    DRIVE_POWER + correction,
                    DRIVE_POWER - correction,
                    DRIVE_POWER + correction,
                    DRIVE_POWER - correction
            );

            log.debug("Phase", "Approaching");
            log.addData("Target Area (ta)", ta);
            log.addData("Target Area Goal", TARGET_AREA_CLOSE);
            log.addData("tx correction", correction);
            log.update(telemetry);
        }

        log.debug("Phase", "Arrived!");
        log.update(telemetry);
        log.save();

        limelight.stop();
    }

    // --- Returns tag 11 if visible, null otherwise ---
    private LLResultTypes.FiducialResult getTag11() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == TARGET_TAG_ID) return tag;
        }
        return null;
    }
//k
    // frontLeft, frontRight, backLeft, backRight
    private void setMotors(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}

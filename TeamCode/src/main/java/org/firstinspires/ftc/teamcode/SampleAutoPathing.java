package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * SampleAutoPathing
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   The original combined autonomous: optionally searches for AprilTag 11 using the
 *   Limelight 3A camera, then runs the five-leg triangle path
 *   (start -> top -> left -> right -> top -> start) and stops cleanly.
 *   Feature switches at the top of the class (RUN_PATHS, RUN_APRILTAG) let you
 *   enable or disable each phase without deleting code.
 *
 * WHAT IT'S FOR:
 *   The "master" autonomous combining Limelight vision with Pedro Pathing paths.
 *   For isolated testing use SampleAutoPathingFiveLegs (paths only) or
 *   SampleAutoTriangleAprilTag (improved combined version with bug fixes).
 *
 * FEATURE FLAGS (set these at the top of the class):
 *   RUN_APRILTAG = false — skips the vision phase entirely (currently disabled
 *                          due to a null-check ordering bug in aprilTagUpdate()).
 *   RUN_PATHS    = true  — runs the five-leg triangle after the vision phase.
 *
 * HARDWARE REQUIRED (names must match the Driver Hub robot config):
 *   - "front_left_motor", "front_right_motor", "back_left_motor", "back_right_motor"
 *   - "pinpoint"  (goBILDA Pinpoint odometry computer)
 *   - "limelight" (Limelight 3A camera, AprilTag pipeline 0)
 * =====================================================================================
 */

import android.util.Log;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Drive Forward")
public class SampleAutoPathing extends OpMode {
    private static final String TAG = "SampleAutoPathing";
    private static final boolean RUN_PATHS = true;
    private Timer aprilTagTimer;
    private double lastUpdateTime = 0;
    private Limelight3A limelight;
    private static final boolean RUN_APRILTAG = false;
    private static final int TARGET_TAG_ID = 11;
    private boolean aprilTagDone = false;



    private Follower follower;
    private TelemetryLog telemetryLog;
    private Timer pathTimer;

    private final Pose startPose = new Pose(71, 62, 0);
    private final Pose topPose  = new Pose(131, 62, 0);
    private final Pose leftPose = new Pose(79, 92, 0);
    private final Pose rightPose = new Pose(79, 32, 0);

    private PathChain leg1;
    private PathChain leg2;
    private PathChain leg3;
    private PathChain leg4;
    private PathChain leg5;

    private boolean leg1Started = false;
    private boolean leg2Started = false;
    private boolean leg3Started = false;
    private boolean leg4Started = false;
    private boolean leg5Started = false;

    @Override
    public void init() {
        Log.i(TAG, "init() started");
        telemetryLog = new TelemetryLog(PanelsTelemetry.INSTANCE.getTelemetry(), "SampleAutoPathing");
        aprilTagTimer = new Timer();

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        follower.setMaxPower(0.75);// 1/4 speed

        leg1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, topPose))
                .setConstantHeadingInterpolation(0)
                .build();
        leg2 = follower.pathBuilder()
                .addPath(new BezierLine(topPose, leftPose))
                .setConstantHeadingInterpolation(0)
                .build();
        leg3 = follower.pathBuilder()
                .addPath(new BezierLine(leftPose, rightPose))
                .setConstantHeadingInterpolation(0)
                .build();
        leg4 = follower.pathBuilder()
                .addPath(new BezierLine(rightPose, topPose))
                .setConstantHeadingInterpolation(0)
                .build();

        leg5 = follower.pathBuilder()
                .addPath(new BezierLine(topPose, startPose))
                .setConstantHeadingInterpolation(0)
                .build();

        follower.setPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
        follower.startTeleopDrive();
        Log.i(TAG, "init() complete");
    }
    private void aprilTagUpdate() {

        if (!RUN_APRILTAG || aprilTagDone) {
            return;
        }

        LLResult result = limelight.getLatestResult();

        // DEBUG â€” show everything the camera sees
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        telemetryLog.addData("Tags seen", fiducials.size());
        for (LLResultTypes.FiducialResult f : fiducials) {
            telemetryLog.debug("Tag ID", String.valueOf(f.getFiducialId()));
        }

        if (result == null || !result.isValid()) {
            follower.setTeleOpDrive(0.0, 0.0, 0.20, true);
            telemetryLog.debug("Status", "Scanning...");
            Log.d(TAG, "aprilTag: no valid result, scanning");
            return;
        }

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == TARGET_TAG_ID) {
                target = f;
                break;
            }
        }

        if (target == null) {
            follower.setTeleOpDrive(0.0, 0.0, 0.20, true);
            telemetryLog.debug("Status", "Scanning for tag " + TARGET_TAG_ID);
            Log.d(TAG, "aprilTag: tag " + TARGET_TAG_ID + " not in frame");
            return;
        }

        double tx = target.getTargetXDegrees();
        double distMeters = target.getRobotPoseTargetSpace().getPosition().y;
        double distInches = Math.abs(distMeters) * 39.3701;
        Log.d(TAG, "aprilTag: tag " + TARGET_TAG_ID + " found â€” tx=" + tx + " dist=" + distInches + "in");

        telemetryLog.addData("TX", tx);
        telemetryLog.addData("Dist inches", distInches);

        // Step 1: Align â€” turn until tx is near zero
        if (Math.abs(tx) > 1.5) {
            double turnPower = tx * 0.03;
            turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
            follower.setTeleOpDrive(0.0, 0.0, turnPower, true);
            telemetryLog.debug("Status", "Aligning...");
            return;
        }

        // Step 2: Drive forward until 12 inches away
        if (distInches > 13.0) {
            double drivePower = (distInches - 12.0) * 0.02;
            drivePower = Math.max(-0.3, Math.min(0.3, drivePower));
            double steer = tx * 0.02;
            follower.setTeleOpDrive(drivePower, 0.0, steer, true);
            telemetryLog.debug("Status", "Approaching...");
            return;
        }

        // Step 3: Parked
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
        aprilTagDone = true;
        Log.i(TAG, "aprilTag: PARKED 12 inches from tag " + TARGET_TAG_ID);
        telemetryLog.debug("Status", "Parked 12 inches from tag!");
    }

    @Override
    public void start() {
        Log.i(TAG, "start()");
        pathTimer.resetTimer();

        // ---- LOGCAT DEMO: the five log levels, lowest to highest severity ----
        // In Logcat each level is color-coded. Filter by tag:SampleAutoPathing
        // (or by level, e.g. level:warn) to see these.
        Log.v(TAG, "VERBOSE: most detailed, noisy stuff you rarely need");
        Log.d(TAG, "DEBUG: values useful while developing, e.g. startPose=" + startPose);
        Log.i(TAG, "INFO: normal milestones, e.g. OpMode started");
        Log.w(TAG, "WARN: something looks off but we can continue");
        Log.e(TAG, "ERROR: something actually went wrong");

        // You can also log a full exception with its stack trace like this:
        try {
            throw new RuntimeException("demo exception â€” this is intentional, not a real bug");
        } catch (Exception e) {
            Log.e(TAG, "ERROR with stack trace:", e);
        }
        // ---------------------------------------------------------------------
    }

    @Override
    public void loop() {
        follower.update();

        if (!aprilTagDone) {
            aprilTagUpdate();
        }
        else if (RUN_PATHS) {

            if (!leg1Started) {
                Log.i(TAG, "path: starting leg1");
                follower.followPath(leg1, true);
                leg1Started = true;
            }
            if (!follower.isBusy() && !leg2Started) {
                Log.i(TAG, "path: starting leg2");
                follower.followPath(leg2, true);
                leg2Started = true;
            }
            if (!follower.isBusy() && !leg3Started && leg2Started) {
                Log.i(TAG, "path: starting leg3");
                follower.followPath(leg3, true);
                leg3Started = true;
            }
            if (!follower.isBusy() && !leg4Started && leg3Started) {
                Log.i(TAG, "path: starting leg4");
                follower.followPath(leg4, true);
                leg4Started = true;
            }
            if (!follower.isBusy() && !leg5Started && leg4Started) {
                Log.i(TAG, "path: starting leg5");
                follower.followPath(leg5, true);
                leg5Started = true;
            }
            if (leg5Started && !follower.isBusy()) {
                follower.breakFollowing();
            }
        }
        telemetryLog.addData("X", follower.getPose().getX());
        telemetryLog.addData("Y", follower.getPose().getY());
        telemetryLog.debug("Busy", String.valueOf(follower.isBusy()));
        telemetryLog.update(telemetry);

        // throttled logcat: once per second
        if (pathTimer.getElapsedTimeSeconds() - lastUpdateTime >= 1.0) {
            lastUpdateTime = pathTimer.getElapsedTimeSeconds();
            Log.d(TAG, String.format("loop @%.0fs  x=%.1f y=%.1f busy=%b",
                    pathTimer.getElapsedTimeSeconds(),
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.isBusy()));
        }
    }

    @Override
    public void stop() {
        telemetryLog.save();
    }
}

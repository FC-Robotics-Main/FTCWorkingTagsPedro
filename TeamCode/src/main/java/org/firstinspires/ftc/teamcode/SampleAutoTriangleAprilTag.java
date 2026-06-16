package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "Triangle + AprilTag")
public class SampleAutoTriangleAprilTag extends OpMode {

    private static final String TAG = "SampleAutoTriangleAprilTag";

    // ---------------------------------------------------------------------
    // Java's equivalent of #ifdef conditional compilation.
    // When a `static final boolean` is false, the compiler performs dead-code
    // elimination on the guarded blocks — they're effectively compiled out,
    // just like a preprocessor switch in C/C++.
    // ---------------------------------------------------------------------
    private static final boolean RUN_PATHS    = true;   // run the 5-leg triangle
    private static final boolean RUN_APRILTAG  = true;   // align to the AprilTag first
    private static final boolean RUN_LOG_DEMO  = false;  // print the logcat level demo in start()

    private static final int TARGET_TAG_ID = 11;

    private Follower follower;
    private TelemetryLog telemetryLog;
    private Timer pathTimer;
    private double lastUpdateTime = 0;

    private Limelight3A limelight;
    private Timer aprilTagTimer;
    private boolean aprilTagDone = false;
    private boolean tagAcquired = false;  // latched true once the target tag is first seen

    private final Pose startPose = new Pose(71, 62, 0);
    private final Pose topPose   = new Pose(131, 62, 0);
    private final Pose leftPose  = new Pose(79, 92, 0);
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

    private PathChain activeLeg = null;

    @Override
    public void init() {
        Log.i(TAG, "init() started");
        telemetryLog = new TelemetryLog(PanelsTelemetry.INSTANCE.getTelemetry(), TAG);
        pathTimer = new Timer();
        aprilTagTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.5);

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

        if (RUN_APRILTAG) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
            follower.startTeleopDrive();
        } else {
            // No tag phase — paths can start immediately.
            aprilTagDone = true;
        }

        Log.i(TAG, "init() complete");
    }

    private void aprilTagUpdate() {
        if (aprilTagDone) {
            return;
        }

        LLResult result = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducials =
                (result != null) ? result.getFiducialResults() : null;
        telemetryLog.addData("Tags seen", fiducials != null ? fiducials.size() : 0);
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult f : fiducials) {
                telemetryLog.debug("Tag ID", String.valueOf(f.getFiducialId()));
            }
        }

        // Find the target tag in this frame, if present and valid.
        LLResultTypes.FiducialResult target = null;
        if (result != null && result.isValid() && fiducials != null) {
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f.getFiducialId() == TARGET_TAG_ID) {
                    target = f;
                    break;
                }
            }
        }

        // No usable detection this frame.
        if (target == null) {
            if (!tagAcquired) {
                // Never seen it yet — sweep slowly to search.
                follower.setTeleOpDrive(0.0, 0.0, 0.15, true);
                telemetryLog.debug("Status", "Scanning for tag " + TARGET_TAG_ID);
            } else {
                // Already locked on once but lost it — sweep slowly to find it
                // again rather than freezing in place.
                follower.setTeleOpDrive(0.0, 0.0, 0.12, true);
                telemetryLog.debug("Status", "Reacquiring...");
            }
            return;
        }

        tagAcquired = true;
        double tx = target.getTargetXDegrees();
        double distMeters = target.getRobotPoseTargetSpace().getPosition().y;
        double distInches = Math.abs(distMeters) * 39.3701;

        telemetryLog.addData("TX", tx);
        telemetryLog.addData("Dist inches", distInches);

        // Step 1: Align — turn until tx is near zero.
        // Lower gain + tighter clamp so it eases in instead of overshooting.
        if (Math.abs(tx) > 1.5) {
            // Negative sign: turn TOWARD the tag (previous version turned away
            // and pushed the tag out of frame).
            double turnPower = -tx * 0.02;
            turnPower = Math.max(-0.20, Math.min(0.20, turnPower));
            // Minimum power so it doesn't stall just outside the deadband.
            if (Math.abs(turnPower) < 0.06) {
                turnPower = Math.copySign(0.06, turnPower);
            }
            follower.setTeleOpDrive(0.0, 0.0, turnPower, true);
            telemetryLog.debug("Status", "Aligning...");
            return;
        }

        // Step 2: Drive forward until 12 inches away
        if (distInches > 13.0) {
            // Negative: positive teleop-forward was driving the robot backward.
            double drivePower = -(distInches - 12.0) * 0.02;
            drivePower = Math.max(-0.3, Math.min(0.3, drivePower));
            double steer = -tx * 0.02;  // same sign fix as alignment
            follower.setTeleOpDrive(drivePower, 0.0, steer, true);
            telemetryLog.debug("Status", "Approaching...");
            return;
        }

        // Step 3: Parked — hand off to the path phase
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
        aprilTagDone = true;
        Log.i(TAG, "aprilTag: PARKED 12 inches from tag " + TARGET_TAG_ID);
        telemetryLog.debug("Status", "Parked 12 inches from tag!");
    }

    @Override
    public void start() {
        Log.i(TAG, "start()");
        pathTimer.resetTimer();

        if (RUN_LOG_DEMO) {
            Log.v(TAG, "VERBOSE: most detailed, noisy stuff you rarely need");
            Log.d(TAG, "DEBUG: values useful while developing, e.g. startPose=" + startPose);
            Log.i(TAG, "INFO: normal milestones, e.g. OpMode started");
            Log.w(TAG, "WARN: something looks off but we can continue");
            Log.e(TAG, "ERROR: something actually went wrong");
        }
    }

    @Override
    public void loop() {
        if (!aprilTagDone) {
            follower.update();
            aprilTagUpdate();
        } else if (RUN_PATHS) {
            if (leg5Started && !follower.isBusy()) {
                follower.breakFollowing();
            } else {
                follower.update();

                // If the robot gets stuck mid-path, re-follow the current leg
                if (follower.isRobotStuck() && activeLeg != null) {
                    follower.followPath(activeLeg, true);
                }

                if (!leg1Started) {
                    activeLeg = leg1;
                    follower.followPath(leg1, true);
                    leg1Started = true;
                }
                if (!follower.isBusy() && leg1Started && !leg2Started) {
                    activeLeg = leg2;
                    follower.followPath(leg2, true);
                    leg2Started = true;
                }
                if (!follower.isBusy() && leg2Started && !leg3Started) {
                    activeLeg = leg3;
                    follower.followPath(leg3, true);
                    leg3Started = true;
                }
                if (!follower.isBusy() && leg3Started && !leg4Started) {
                    activeLeg = leg4;
                    follower.followPath(leg4, true);
                    leg4Started = true;
                }
                if (!follower.isBusy() && leg4Started && !leg5Started) {
                    activeLeg = leg5;
                    follower.followPath(leg5, true);
                    leg5Started = true;
                }
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

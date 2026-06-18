package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * SampleAutoTriangleAprilTag
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   An autonomous that combines Limelight AprilTag tracking with Pedro Pathing.
 *
 *   1) APRILTAG PHASE:
 *        a. SCAN   — spins slowly until tag #11 is visible.
 *        b. ALIGN  — rotates until the tag is centered (tx ~ 0°).
 *        c. APPROACH — drives toward the tag until it fills TARGET_AREA_CLOSE % of
 *                      the camera image, then stops. ≈ 1 foot for a standard 6.5"
 *                      tag at 35% — tune this constant for your setup.
 *   2) TRIANGLE PHASE: After parking, rebuilds leg1 from the robot's current position
 *      (tracked by Pinpoint throughout the vision phase) and runs the 5-leg triangle.
 *
 * TUNING:
 *   TARGET_AREA_CLOSE — watch "TA (%)" on DS while robot approaches tag; note the
 *   value at your desired stop distance and set this constant to match.
 *
 * HARDWARE REQUIRED:
 *   "front_left_motor", "front_right_motor", "back_left_motor", "back_right_motor"
 *   "pinpoint"  (goBILDA Pinpoint)
 *   "limelight" (Limelight 3A, AprilTag pipeline 0)
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Triangle + AprilTag")
public class SampleAutoTriangleAprilTag extends OpMode {

    private static final String TAG = "SampleAutoTriangleAprilTag";

    // ── Feature switches ──────────────────────────────────────────────────────
    private static final boolean RUN_PATHS    = true;
    private static final boolean RUN_APRILTAG = true;
    private static final boolean RUN_LOG_DEMO = false;

    private static final int TARGET_TAG_ID = 11;

    // Stop approaching when the tag fills this % of the camera image.
    // Watch "TA max" on the Driver Station: it shows the largest ta seen before the
    // tag went out of view. Set this constant to that value minus a small margin so
    // the robot stops just before losing sight of the tag (~1 foot away).
    private static final double TARGET_AREA_CLOSE = 55.0;

    // SQUARE-UP: if true, the robot strafes onto the tag's centerline before
    // approaching, so it ends up directly in front even when it arrives diagonally.
    // Uses getRobotPoseTargetSpace() (the tag-relative 3D pose). If that data is
    // noisy on your setup, set this false to fall back to the old point-and-drive.
    private static final boolean USE_SQUAREUP   = true;
    // Robot is "centered" when its lateral offset from the tag centerline is under
    // this many inches (from robot-in-target-space position.x).
    private static final double  CENTER_TOL_IN  = 2.0;
    private static final double  STRAFE_GAIN    = 0.04;   // power per inch of lateral error
    private static final double  STRAFE_MAX     = 0.30;

    // ── Pedro Pathing ─────────────────────────────────────────────────────────
    private Follower    follower;
    private TelemetryLog telemetryLog;
    private Timer       pathTimer;
    private double      lastUpdateTime = 0;

    // ── Vision ────────────────────────────────────────────────────────────────
    private Limelight3A limelight;
    private boolean     aprilTagDone  = false;
    private boolean     tagAcquired   = false;
    private boolean     alignedOnce   = false; // latched true once robot first faces the tag
    private boolean     squaredUp     = false; // latched true once on the tag's centerline
    private double      maxTaSeen    = 0.0;   // highest ta recorded during approach
    private long        tagLostAtMs  = -1L;   // ms timestamp when tag was lost, -1 = visible
    private Pose        tagLostPose  = null;  // robot pose at the moment the tag was lost
    private boolean     needReturn   = false; // true when timeout path back is required
    private PathChain   returnPath   = null;
    private boolean     returnStarted = false;
    private boolean     returnDone   = false;

    // ── Triangle waypoints ────────────────────────────────────────────────────
    private final Pose startPose = new Pose(71,  62, 0);
    private final Pose topPose   = new Pose(131, 62, 0);
    private final Pose leftPose  = new Pose(79,  92, 0);
    private final Pose rightPose = new Pose(79,  32, 0);

    // ── Path segments ─────────────────────────────────────────────────────────
    private PathChain leg1, leg2, leg3, leg4, leg5;
    private boolean   leg1Started = false, leg2Started = false,
                      leg3Started = false, leg4Started = false, leg5Started = false;
    private PathChain activeLeg = null;

    // Rebuilt from real current pose when vision phase ends.
    private boolean pathPhaseInitialized = false;

    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init() {
        Log.i(TAG, "init()");

        telemetryLog = new TelemetryLog(PanelsTelemetry.INSTANCE.getTelemetry(), TAG);
        pathTimer    = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.5);

        // Build triangle legs. leg1 is rebuilt from real pose at vision-phase end.
        leg1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, topPose))
                .setConstantHeadingInterpolation(0).build();
        leg2 = follower.pathBuilder()
                .addPath(new BezierLine(topPose, leftPose))
                .setConstantHeadingInterpolation(0).build();
        leg3 = follower.pathBuilder()
                .addPath(new BezierLine(leftPose, rightPose))
                .setConstantHeadingInterpolation(0).build();
        leg4 = follower.pathBuilder()
                .addPath(new BezierLine(rightPose, topPose))
                .setConstantHeadingInterpolation(0).build();
        leg5 = follower.pathBuilder()
                .addPath(new BezierLine(topPose, startPose))
                .setConstantHeadingInterpolation(0).build();

        follower.setPose(startPose);

        if (RUN_APRILTAG) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
            // Use Pedro's teleop-drive mode so setTeleOpDrive() controls the motors.
            follower.startTeleopDrive();
        } else {
            aprilTagDone = true;
        }

        Log.i(TAG, "init() complete");
    }

    /**
     * Vision phase: scan → simultaneous align+approach → park.
     *
     * Turn and drive corrections are applied in the SAME command so the robot
     * never has to stop forward motion just to re-align. This eliminates the
     * twitching that occurs when sequential steps fight each other near the target.
     */
    private void aprilTagUpdate() {
        if (aprilTagDone) return;

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials =
                (result != null) ? result.getFiducialResults() : null;

        telemetryLog.addData("Tags seen", fiducials != null ? fiducials.size() : 0);

        LLResultTypes.FiducialResult target = null;
        if (result != null && result.isValid() && fiducials != null) {
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f.getFiducialId() == TARGET_TAG_ID) { target = f; break; }
            }
        }

        // ── Tag not visible ────────────────────────────────────────────────
        if (target == null) {
            if (!tagAcquired) {
                // Never seen the tag yet — keep scanning.
                follower.setTeleOpDrive(0.0, 0.0, 0.15, true);
                telemetryLog.debug("Phase", "SCAN – looking for tag " + TARGET_TAG_ID);
            } else {
                // Saw the tag but lost it. Spin slowly to reacquire; if still gone
                // after 2 seconds, return to where the tag was lost then do the triangle.
                if (tagLostAtMs < 0) {
                    tagLostAtMs = System.currentTimeMillis();
                    tagLostPose = follower.getPose();   // snapshot position right now
                }
                long lostForMs = System.currentTimeMillis() - tagLostAtMs;
                if (lostForMs > 2000) {
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
                    aprilTagDone = true;
                    needReturn   = true;
                    Log.i(TAG, String.format(
                            "Lost tag for >2 s — returning to (%.1f, %.1f) then triangle",
                            tagLostPose.getX(), tagLostPose.getY()));
                    telemetryLog.debug("Phase", "TIMEOUT – returning to lost pose");
                } else {
                    follower.setTeleOpDrive(0.0, 0.0, 0.12, true);
                    telemetryLog.debug("Phase",
                            String.format("REACQUIRE (%.1f s)", lostForMs / 1000.0));
                }
            }
            return;
        }

        // Tag is visible — reset lost timer.
        tagLostAtMs = -1L;

        tagAcquired = true;
        double tx = target.getTargetXDegrees();  // °, 0 = centered
        double ta = target.getTargetArea();       // %, larger = closer
        maxTaSeen = Math.max(maxTaSeen, ta);

        telemetryLog.addData("TX  (deg)", tx);
        telemetryLog.addData("TA  (%)",   ta);
        telemetryLog.addData("TA goal",   TARGET_AREA_CLOSE);
        telemetryLog.addData("TA max",    maxTaSeen);

        // ── Parked: both conditions met → hand off to path phase ──────────
        if (ta >= TARGET_AREA_CLOSE && Math.abs(tx) <= 2.0) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
            aprilTagDone = true;
            Log.i(TAG, String.format("PARKED (ta=%.1f%%, tx=%.1f°)", ta, tx));
            telemetryLog.debug("Phase", String.format("PARKED (ta=%.1f%%)", ta));
            return;
        }

        // ── PRE-ALIGN: rotate until squarely facing the tag before any forward drive ──
        // Latches open once the robot is within ±2° — prevents diagonal approach.
        if (!alignedOnce) {
            if (Math.abs(tx) > 2.0) {
                double turnPower = -tx * 0.02;
                turnPower = Math.max(-0.20, Math.min(0.20, turnPower));
                if (Math.abs(turnPower) < 0.06) turnPower = Math.copySign(0.06, turnPower);
                follower.setTeleOpDrive(0.0, 0.0, turnPower, true);
                telemetryLog.debug("Phase", String.format("PRE-ALIGN tx=%.1f°", tx));
                return;
            }
            alignedOnce = true;
        }

        // ── SQUARE-UP: strafe onto the tag's centerline so the robot ends up ──
        // directly in front, not approaching diagonally. Reads the robot's pose in
        // the tag's frame; position.x is the lateral offset from the centerline.
        // Strafe to zero it while turning to keep the tag centered (an orbit onto
        // the front). Latches once centered so it never re-triggers near the tag.
        if (USE_SQUAREUP && !squaredUp) {
            Pose3D rp = target.getRobotPoseTargetSpace();
            Position pos = (rp != null) ? rp.getPosition().toUnit(DistanceUnit.INCH) : null;

            if (pos != null) {
                double lateralIn = pos.x;                    // +x = robot right of center
                double yawDeg = rp.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetryLog.addData("Lateral (in)", lateralIn);
                telemetryLog.addData("Depth Z (in)", pos.z);
                telemetryLog.addData("Yaw (deg)",    yawDeg);

                if (Math.abs(lateralIn) > CENTER_TOL_IN) {
                    // Strafe toward centerline. If it strafes the WRONG way on your
                    // robot, flip the sign of STRAFE_GAIN.
                    double strafe = -lateralIn * STRAFE_GAIN;
                    strafe = Math.max(-STRAFE_MAX, Math.min(STRAFE_MAX, strafe));
                    if (Math.abs(strafe) < 0.08) strafe = Math.copySign(0.08, strafe);
                    // Keep tag centered in view while strafing (tx sign is reliable).
                    double keepCentered = -tx * 0.02;
                    keepCentered = Math.max(-0.15, Math.min(0.15, keepCentered));
                    follower.setTeleOpDrive(0.0, strafe, keepCentered, true);
                    telemetryLog.debug("Phase",
                            String.format("SQUARE-UP lat=%.1fin tx=%.1f°", lateralIn, tx));
                    return;
                }
                squaredUp = true;   // on the centerline — proceed to approach
            } else {
                // No valid tag-relative pose this frame; just hold heading and wait.
                follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
                telemetryLog.debug("Phase", "SQUARE-UP (waiting for pose)");
                return;
            }
        }

        // ── Turn correction (continuous steering during approach) ─────────
        double turnPower = 0.0;
        if (Math.abs(tx) > 1.5) {
            turnPower = -tx * 0.02;
            turnPower = Math.max(-0.20, Math.min(0.20, turnPower));
            if (Math.abs(turnPower) < 0.06) turnPower = Math.copySign(0.06, turnPower);
        }

        // ── Drive correction (approach while roughly centered) ────────────
        // Pause forward drive only if grossly off-axis (>20°).
        double drivePower = 0.0;
        if (ta < TARGET_AREA_CLOSE && Math.abs(tx) < 20.0) {
            double gap = TARGET_AREA_CLOSE - ta;
            drivePower = gap * 0.01;
            drivePower = Math.min(0.30, drivePower);
            drivePower = Math.max(0.10, drivePower);
        }

        follower.setTeleOpDrive(drivePower, 0.0, turnPower, true);

        String phase = (drivePower != 0.0) ? "APPROACH+STEER" : "STEER ONLY";
        telemetryLog.debug("Phase", String.format("%s tx=%.1f° ta=%.1f%%", phase, tx, ta));
    }

    @Override
    public void start() {
        Log.i(TAG, "start()");
        pathTimer.resetTimer();
        if (RUN_LOG_DEMO) {
            Log.v(TAG, "VERBOSE"); Log.d(TAG, "DEBUG");
            Log.i(TAG, "INFO");   Log.w(TAG, "WARN"); Log.e(TAG, "ERROR");
        }
    }

    @Override
    public void loop() {
        if (!aprilTagDone) {
            // ── VISION PHASE ──────────────────────────────────────────────
            follower.update();    // keeps Pinpoint odometry current
            aprilTagUpdate();

        } else if (RUN_PATHS) {

            // ── RETURN PHASE (only when vision timed out) ─────────────────
            // Drive back to the position the robot was at when it lost the tag,
            // then fall through into the triangle on the next iteration.
            if (needReturn && !returnDone) {
                if (returnPath == null) {
                    Pose here = follower.getPose();
                    double dx = tagLostPose.getX() - here.getX();
                    double dy = tagLostPose.getY() - here.getY();
                    if (Math.sqrt(dx * dx + dy * dy) < 2.0) {
                        // Already essentially at the lost pose — skip the path.
                        returnDone = true;
                    } else {
                        returnPath = follower.pathBuilder()
                                .addPath(new BezierLine(here, tagLostPose))
                                .setConstantHeadingInterpolation(tagLostPose.getHeading())
                                .build();
                    }
                }
                if (!returnDone) {
                    follower.update();
                    if (!returnStarted) {
                        follower.followPath(returnPath, true);
                        returnStarted = true;
                    }
                    if (!follower.isBusy()) returnDone = true;
                    telemetryLog.debug("Phase", "RETURNING to lost pose");
                }

            // ── TRIANGLE PHASE ────────────────────────────────────────────
            } else {
                // First entry: the vision phase left the follower in teleop mode with
                // an uncertain odometry frame (free spinning + approach). Reset to a
                // clean known pose so the legs complete exactly like the standalone
                // triangle. The triangle is then drawn relative to where the robot
                // physically parked / is facing.
                if (!pathPhaseInitialized) {
                    follower.breakFollowing();   // exit teleop mode, clear hold state
                    // Reset to startPose position but heading = π. leg1 holds heading 0,
                    // so the robot does the 180° turn (away from the tag, toward the
                    // field) before driving the triangle. Pose and path share one frame
                    // so the legs still complete reliably.
                    follower.setPose(new Pose(startPose.getX(), startPose.getY(), Math.PI));
                    pathPhaseInitialized = true;
                    Log.i(TAG, "Triangle: reset frame (heading=180°), starting legs");
                }

                if (leg5Started && !follower.isBusy()) {
                    follower.breakFollowing();
                } else {
                    follower.update();

                    if (follower.isRobotStuck() && activeLeg != null) {
                        follower.followPath(activeLeg, true);
                    }
                    if (!leg1Started) {
                        activeLeg = leg1; follower.followPath(leg1, true); leg1Started = true;
                    }
                    if (!follower.isBusy() && leg1Started && !leg2Started) {
                        activeLeg = leg2; follower.followPath(leg2, true); leg2Started = true;
                    }
                    if (!follower.isBusy() && leg2Started && !leg3Started) {
                        activeLeg = leg3; follower.followPath(leg3, true); leg3Started = true;
                    }
                    if (!follower.isBusy() && leg3Started && !leg4Started) {
                        activeLeg = leg4; follower.followPath(leg4, true); leg4Started = true;
                    }
                    if (!follower.isBusy() && leg4Started && !leg5Started) {
                        activeLeg = leg5; follower.followPath(leg5, true); leg5Started = true;
                    }
                }
            }
        }

        // ── Telemetry ─────────────────────────────────────────────────────
        telemetryLog.addData("X", follower.getPose().getX());
        telemetryLog.addData("Y", follower.getPose().getY());
        telemetryLog.debug("Busy", String.valueOf(follower.isBusy()));
        telemetryLog.update(telemetry);

        if (pathTimer.getElapsedTimeSeconds() - lastUpdateTime >= 1.0) {
            lastUpdateTime = pathTimer.getElapsedTimeSeconds();
            Log.d(TAG, String.format("@%.0fs  x=%.1f y=%.1f busy=%b",
                    pathTimer.getElapsedTimeSeconds(),
                    follower.getPose().getX(), follower.getPose().getY(),
                    follower.isBusy()));
        }
    }

    @Override
    public void stop() {
        telemetryLog.save();
    }
}

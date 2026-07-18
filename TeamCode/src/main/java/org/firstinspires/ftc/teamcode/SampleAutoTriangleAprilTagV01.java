package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * SampleAutoTriangleAprilTagV01  —  VERSION SNAPSHOT (current working version)
 * -------------------------------------------------------------------------------------
 * This is V01: the V00 baseline plus the 180° handoff fix and the readability pass
 * (documented header, grouped imports, flag-driven op-mode name). To make further
 * changes, copy this to V02 and edit that — never clobber a lower version.
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   An autonomous that combines Limelight AprilTag tracking with Pedro Pathing.
 *
 *   1) APRILTAG PHASE:
 *        a. SCAN      — spins slowly in place until tag #11 is visible.
 *        b. PRE-ALIGN — rotates until the tag is centered (TX within ±2°).
 *        c. SQUARE-UP — strafes sideways onto the tag's centerline so the robot
 *                       ends up directly in front of the tag, not off at an angle.
 *        d. APPROACH  — drives toward the tag, steering to keep it centered, until
 *                       it fills TARGET_AREA_CLOSE % of the camera image (TA).
 *        e. PARK      — stops there (roughly 1 foot away with the current measured
 *                       value; see the TARGET_AREA_CLOSE comment for how it was tuned).
 *      SAFETY NET: if the tag is lost from view for more than 2 seconds, the robot
 *      gives up, drives back to where it last saw the tag, and moves on to the
 *      triangle anyway.
 *   2) TRIANGLE PHASE: After parking, snaps the odometry back to the fixed start
 *      point but facing 180° (turned away from the tag), then drives the fixed
 *      5-leg triangle from there.
 *
 * VISION TERMS (measurements the Limelight reports about the tag it sees):
 *   TX = "Target X" — the HORIZONTAL angle, in degrees, from the center of the
 *        camera image to the tag. TX = 0° means the tag is dead-center; negative
 *        = tag is to the left, positive = tag is to the right. We rotate the robot
 *        to drive TX toward 0 so it faces the tag squarely.
 *        (from FiducialResult.getTargetXDegrees())
 *   TA = "Target Area" — how much of the camera image the tag fills, as a
 *        PERCENT (0-100). The closer the robot is to the tag, the bigger the tag
 *        looks, so TA rises as we approach. We use it as a stand-in for distance:
 *        stop driving once TA reaches TARGET_AREA_CLOSE.
 *        (from FiducialResult.getTargetArea())
 *
 * TUNING:
 *   TARGET_AREA_CLOSE — watch "TA (%)" on the Driver Station while the robot
 *   approaches the tag; note the value at your desired stop distance and set this
 *   constant to match.
 *
 * HARDWARE REQUIRED:
 *   IMPORTANT: Each name in quotes below must match the device name in your robot
 *   configuration on the Driver Station EXACTLY — same spelling, same lowercase,
 *   same underscores. The code looks each device up by this exact string, so if
 *   the configuration says "frontLeft" but the code says "front_left_motor" (or
 *   vice-versa), the op mode will crash on start with a "device not found" error.
 *   To fix a mismatch, either rename the device in the Driver Station configuration
 *   or change the string here so the two are identical.
 *
 *   "front_left_motor", "front_right_motor", "back_left_motor", "back_right_motor"
 *   "pinpoint"  (goBILDA Pinpoint)
 *   "limelight" (Limelight 3A, AprilTag pipeline 0)
 * =====================================================================================
 */

// #############################################################################
// DO NOT RUN "OPTIMIZE IMPORTS" (Ctrl+Alt+O) ON THIS FILE.
// The imports below are deliberately grouped by function with explanatory
// comments. Android Studio's Optimize Imports re-sorts them alphabetically and
// STRIPS every comment and blank line here, undoing all of it. If you add a new
// import, place it in the matching group by hand instead.
// #############################################################################

// ─────────────────────────────────────────────────────────────────────────────
// FTC SDK — OP-MODE FRAMEWORK
// The base pieces that turn this class into a program the robot can run.
// ─────────────────────────────────────────────────────────────────────────────
// Base class we extend; gives us the init() / loop() / stop() lifecycle.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// Annotation that lists this op mode under Autonomous on the Driver Station.
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// ─────────────────────────────────────────────────────────────────────────────
// GENERAL UTILITIES (Android + Java)
// Not robot-specific; basic logging and collection helpers.
// ─────────────────────────────────────────────────────────────────────────────
// Writes debug messages to Android's logcat (viewable over ADB / Logcat).
import android.util.Log;
// Holds the list of AprilTags the camera reports in a single frame.
import java.util.List;

// ─────────────────────────────────────────────────────────────────────────────
// LIMELIGHT APRILTAG VISION
// Reading AprilTags from the Limelight 3A camera during the vision phase.
// ─────────────────────────────────────────────────────────────────────────────
// Heads-up on these names: "LL" is short for Limelight (the camera's brand), and a
// "fiducial" is just the formal word for an AprilTag — a printed square marker the
// camera is built to recognize. We can't rename these classes (they come from the
// Limelight library), so remember: LL... = Limelight..., Fiducial = AprilTag.
//
// The Limelight 3A camera itself: start(), getLatestResult(), pipeline switching.
import com.qualcomm.hardware.limelightvision.Limelight3A;
// LLResult = one snapshot of everything the Limelight saw in a single camera frame.
import com.qualcomm.hardware.limelightvision.LLResult;
// LLResultTypes groups the kinds of things in a result. We use its FiducialResult,
// which is the data for ONE detected AprilTag (its ID, where it is on screen, etc.).
import com.qualcomm.hardware.limelightvision.LLResultTypes;

// ─────────────────────────────────────────────────────────────────────────────
// TAG-RELATIVE 3D GEOMETRY
// Describes where the robot sits in the tag's own coordinate frame; used by the
// square-up phase to tell "directly in front" from "off to one side".
// ─────────────────────────────────────────────────────────────────────────────
// Full 3D position + orientation of the robot relative to the tag.
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
// The x / y / z part of a Pose3D, carrying its distance unit.
import org.firstinspires.ftc.robotcore.external.navigation.Position;
// Lets us request the yaw angle specifically in degrees.
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// Lets us convert the lateral offset into inches.
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// ─────────────────────────────────────────────────────────────────────────────
// PEDRO PATHING
// Odometry tracking and autonomous path following used to drive the triangle.
// ─────────────────────────────────────────────────────────────────────────────
// The path follower: drives the robot along a path and tracks its pose.
import com.pedropathing.follower.Follower;
// A robot position on the field as (x, y, heading).
import com.pedropathing.geometry.Pose;
// A straight-line path segment from one Pose to another.
import com.pedropathing.geometry.BezierLine;
// One or more segments joined into a single followable path.
import com.pedropathing.paths.PathChain;
// Simple elapsed-time timer, used here for once-per-second logging.
import com.pedropathing.util.Timer;
// This robot's Pedro tuning/config; builds a ready-to-use Follower.
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// ─────────────────────────────────────────────────────────────────────────────
// TELEMETRY / LOGGING
// Backend that our TelemetryLog helper writes to (TelemetryLog is in this same
// package, so it needs no import).
// ─────────────────────────────────────────────────────────────────────────────
// The Panels dashboard telemetry backend TelemetryLog sends data to.
import com.bylazar.telemetry.PanelsTelemetry;

// The op-mode name shown on the Driver Station is assembled from the feature
// switches at COMPILE time, so it reflects whatever was enabled in the build (see
// OPMODE_NAME just inside the class). The annotation only needs a constant, so it
// simply points at that one constant — all the readable logic lives below.
@Autonomous(name = SampleAutoTriangleAprilTagV01.OPMODE_NAME)
public class SampleAutoTriangleAprilTagV01 extends OpMode {

    private static final String TAG = "SampleAutoTriangleAprilTagV01";

    // ── Feature switches (poor-man's conditional compilation) ──────────────────
    // What we're really after here is CONDITIONAL COMPILATION — the C/C++ idea of
    // "#ifdef FLAG ... #endif", where code you switched off isn't just skipped at
    // runtime but is left out of the build entirely. Java has no #ifdef and no
    // preprocessor, so there is no direct way to do that.
    //
    // The closest Java equivalent is a `static final boolean` constant. Because its
    // value is fixed at compile time, the compiler treats `if (RUN_APRILTAG) { ... }`
    // specially: when the flag is a constant `false`, the guarded block is
    // recognized as unreachable and effectively dropped from the compiled output,
    // and when it is a constant `true`, the check disappears. So flipping one of
    // these flags and rebuilding behaves much like commenting the feature in or out
    // of the source — that's why they're written as `static final`, not regular
    // variables. (They also drive OPMODE_NAME above for the same compile-time reason.)
    private static final boolean RUN_PATHS    = true;
    private static final boolean RUN_APRILTAG = true;
    private static final boolean RUN_LOG_DEMO = false;

    // Driver Station name, chosen from the switches above.
    //
    // WHY A TERNARY HERE — when we deliberately use if/else everywhere else in this
    // file: the @Autonomous name just above must be a COMPILE-TIME CONSTANT. An
    // if/else is a *statement* — it runs steps and produces no value — so it cannot
    // build a constant. The ternary "condition ? valueA : valueB" is the ONLY
    // conditional in Java that is an *expression* (it evaluates to a single value),
    // so it is the only tool that can choose the name at compile time. This is the
    // one and only place in the file where a ternary is unavoidable; do not "fix" it
    // to an if/else — the code will not compile.
    //
    // It reads as a top-to-bottom decision table — the first true row wins:
    //     vision off   → "Triangle Only"
    //     triangle off → "AprilTag Only"
    //     both on      → "Triangle + AprilTag"
    // then optional suffixes are added. Package-private (not private) so the
    // @Autonomous annotation above is allowed to read it.
    static final String OPMODE_NAME =
            (   !RUN_APRILTAG ? "Triangle Only"          // vision off   → triangle only
              : !RUN_PATHS    ? "AprilTag Only"          // triangle off → AprilTag only
              :                 "Triangle + AprilTag"    // both on      → full routine
            )
            + (RUN_LOG_DEMO ? " [LOG]" : "")             // mark logging-demo builds
            + " V01";                                    // version tag; keeps DS name unique

    // We will only look for AprilTag ID number 11; any other tag in view is ignored.
    private static final long TARGET_TAG_ID = 11;

    // How close is "close enough" to stop, expressed as TA — the percent of the
    // camera image the tag fills (TA grows as the robot gets nearer). When TA
    // reaches this number we stop; here that works out to roughly 1 foot away.
    //
    // Why 55 specifically? It is not a magic constant — it was MEASURED, not
    // calculated. We drove the robot in, watched the "TA max" readout on the Driver
    // Station (the largest TA seen right before the tag left the camera's view), and
    // that peak was about 60. We back off a few percent so the robot stops just
    // BEFORE the tag slips out of frame, which lands at 55. Using 58 would also work
    // but leaves less margin — the robot coasts closer and risks driving under the
    // tag and losing sight of it. Lower it to stop farther back, raise it to stop
    // nearer; re-measure "TA max" if you change the tag size or camera mounting.
    private static final double TARGET_AREA_CLOSE = 55.0;

    // "Squaring up" means lining the robot up so it is directly in front of the
    // tag and facing it head-on, instead of approaching from an angle.
    //
    // Picture a straight line coming out of the face of the tag, perpendicular to
    // it — that's the "centerline". If the robot drives in from the side, it can
    // still be pointed at the tag but sitting off that line, so it reaches the tag
    // crooked. To fix that, the robot slides sideways (a "strafe" — a mecanum drive
    // can move left/right without turning) until it is sitting on that centerline,
    // then drives straight in.
    //
    // To know how far off the line it is, we ask the Limelight for the robot's
    // position measured FROM THE TAG'S point of view (the getRobotPoseTargetSpace()
    // reading). That tells us how many inches left or right of center the robot is.
    //
    // Set this to false to skip all of that and just point-and-drive at the tag
    // (simpler, but it can arrive at an angle). Turn it off if the tag-relative
    // reading is jittery on your camera and causes bad sideways motion.
    private static final boolean USE_SQUAREUP              = true;

    // The robot counts as "on the centerline" once it is within this many inches
    // of dead center, left-to-right. (2.0 inches = close enough; it then stops
    // strafing and starts driving in.)
    private static final double  CENTER_TOLERANCE_INCHES   = 2.0;

    // How hard to strafe per inch of sideways error: bigger = moves over faster
    // but can overshoot; smaller = gentler but slower to center.
    private static final double  STRAFE_GAIN               = 0.04;

    // Upper limit on strafing motor power, so the sideways move never gets violent.
    private static final double  STRAFE_MAX                = 0.30;

    // ── Pedro Pathing (drives the robot, and keeps track of where it is) ────────
    // A few words used all through this file:
    //   Pose     = the robot's spot on the field: its X and Y position PLUS the
    //              direction it is facing (its "heading").
    //   Heading  = which way the robot points, as an angle. Measured in radians here
    //              (radians are just another way to measure angles; π radians = 180°).
    //   Odometry = the robot continuously measuring its own movement so it always has
    //              an up-to-date guess of its Pose. The "follower" below does this.
    private Follower     pathFollower;   // drives along the paths AND tracks the robot's Pose
    private TelemetryLog telemetryLog;   // shows readouts on the dashboard and saves them to a file
    private Timer        elapsedTimer;   // a stopwatch; used to log the position once per second
    private double       fLastUpdateTimeSeconds = 0;  // when we last logged (seconds since start)

    // ── Vision (finding the AprilTag and driving to it) ─────────────────────────
    private Limelight3A  limelightCamera;
    private boolean     bIsAprilTagDone        = false;  // true once the whole camera phase is finished
    private boolean     bIsTagAcquired         = false;  // true once we have seen the tag at least once
    private boolean     bIsAlignedOnce         = false;  // true once the robot has faced the tag (then stays true)
    private boolean     bIsSquaredUp           = false;  // true once the robot is on the tag's centerline (then stays true)
    private long        nSquareUpWaitStartMilliseconds = -1L;  // clock reading (ms) when square-up began waiting for missing camera position data; -1 = not waiting
    private double      fMaxTargetAreaSeen     = 0.0;    // biggest TA seen so far while approaching
    private long        nTagLostAtMilliseconds = -1L;    // clock reading (ms) when the tag was lost; -1 = visible now
    private Pose        tagLostPose            = null;   // where the robot was when the tag was lost
    private boolean     bIsReturnNeeded        = false;  // true if we gave up on the tag and must drive back
    private PathChain   returnPath             = null;   // the path that leads back to tagLostPose
    private boolean     bIsReturnStarted       = false;  // true once we have started driving that return path
    private boolean     bIsReturnDone          = false;  // true once we have finished driving it

    // ── Triangle waypoints (the four corners the robot drives between) ──────────
    private final Pose triangleTop               = new Pose(131, 62, 0);  // apex (top point)
    private final Pose triangleBottomLeft         = new Pose(79,  92, 0);  // left base corner
    private final Pose triangleBottomRight        = new Pose(79,  32, 0);  // right base corner
    private final Pose triangleBelowBottomMiddle  = new Pose(71,  62, 0);  // robot's start, just below the base center

    // ── Path segments: one straight line per side of the trip, each named for the
    //    two corners it connects (leg1StartToTop = start point → top, and so on). ─
    private PathChain leg1StartToTop, leg2TopToLeft, leg3LeftToRight,
                      leg4RightToTop, leg5TopToStart;

    // One flag per leg: becomes true the moment that leg is handed to the follower
    // to drive, so we don't start the same leg twice.
    private boolean   bIsLeg1Started = false, bIsLeg2Started = false,
                      bIsLeg3Started = false, bIsLeg4Started = false, bIsLeg5Started = false;
    private PathChain activeLeg = null;   // the leg the robot is driving right now

    // Becomes true after the one-time setup at the start of the triangle phase
    // (the position reset + 180° turn) has run, so that setup happens only once
    // instead of repeating on every loop.
    private boolean bIsPathPhaseInitialized = false;

    // ─────────────────────────────────────────────────────────────────────────

    /**
     * WHAT: One-time setup for the whole op mode. Builds the telemetry logger and
     *       elapsed-time timer, creates the Pedro Pathing follower, pre-builds the
     *       five triangle legs, sets the robot's starting pose, and — when the
     *       AprilTag phase is enabled — starts the Limelight camera and switches the
     *       follower into teleop-drive mode so the vision phase can steer the motors.
     *
     * CALLED FROM: the FTC SDK automatically (OpMode lifecycle) — runs ONCE when the
     *       driver presses INIT on the Driver Station, before the match starts. We
     *       never call it ourselves.
     */
    @Override
    public void init() {
        // Log.i writes one INFO line to Android's logcat, labeled with TAG so we can
        // filter for this op mode's messages. All the Log.* calls in this file are
        // debugging breadcrumbs only — they do NOT affect the robot. Severity levels,
        // low to high: .v verbose, .d debug, .i info, .w warn, .e error.
        Log.i(TAG, "init()");

        telemetryLog = new TelemetryLog(PanelsTelemetry.INSTANCE.getTelemetry(), TAG);
        elapsedTimer = new Timer();

        pathFollower = Constants.createFollower(hardwareMap);
        pathFollower.setMaxPower(0.5);

        // Pre-build the five triangle legs once, here, from the fixed corners above.
        // They are driven later in loop(); none are rebuilt while the robot is running.
        // Reading one leg below:
        //   new BezierLine(A, B)               = a straight path from point A to point B.
        //   setConstantHeadingInterpolation(0) = keep facing heading 0 the whole way, so
        //                                        the robot slides along without spinning.
        //   build()                            = finish assembling the path.
        leg1StartToTop = pathFollower.pathBuilder()
                .addPath(new BezierLine(triangleBelowBottomMiddle, triangleTop))
                .setConstantHeadingInterpolation(0).build();
        leg2TopToLeft = pathFollower.pathBuilder()
                .addPath(new BezierLine(triangleTop, triangleBottomLeft))
                .setConstantHeadingInterpolation(0).build();
        leg3LeftToRight = pathFollower.pathBuilder()
                .addPath(new BezierLine(triangleBottomLeft, triangleBottomRight))
                .setConstantHeadingInterpolation(0).build();
        leg4RightToTop = pathFollower.pathBuilder()
                .addPath(new BezierLine(triangleBottomRight, triangleTop))
                .setConstantHeadingInterpolation(0).build();
        leg5TopToStart = pathFollower.pathBuilder()
                .addPath(new BezierLine(triangleTop, triangleBelowBottomMiddle))
                .setConstantHeadingInterpolation(0).build();

        pathFollower.setPose(triangleBelowBottomMiddle);

        if (RUN_APRILTAG) {
            // Grab the Limelight camera from the robot configuration (device named "limelight").
            limelightCamera = hardwareMap.get(Limelight3A.class, "limelight");

            // How often to pull fresh data from the camera: 100 times per second (100 Hz).
            limelightCamera.setPollRateHz(100);

            // Pick which on-camera vision "program" to run. A Limelight stores several
            // numbered pipelines; pipeline 0 is the one configured to detect AprilTags.
            limelightCamera.pipelineSwitch(0);

            // Start the camera streaming so getLatestResult() has data to return.
            limelightCamera.start();

            // Put the follower into MANUAL ("teleop") drive mode. Normally the follower
            // steers the robot automatically along a pre-planned path. During the vision
            // phase we instead want to drive by hand based on what the camera sees, so we
            // switch to this mode — after which our setTeleOpDrive(forward, strafe, turn)
            // calls send power straight to the motors. Without this call, setTeleOpDrive
            // would have no effect.
            pathFollower.startTeleopDrive();
        } else {
            bIsAprilTagDone = true;
        }

        Log.i(TAG, "init() complete");
    }

    /**
     * WHAT: Runs one step of the AprilTag vision phase: scan for the tag →
     *       pre-align → square up onto its centerline → approach → park in front.
     *       Turn and drive corrections are applied in the SAME command so the robot
     *       never has to stop forward motion just to re-align, which eliminates the
     *       twitching that happens when sequential steps fight each other near the
     *       target. Sets bIsAprilTagDone = true once parked (or timed out).
     *
     * CALLED FROM: loop(), once per cycle, but only while the vision phase is still
     *       running. This is our own private helper, not part of the OpMode lifecycle.
     */
    private void aprilTagUpdate() {
        if (bIsAprilTagDone) return;

        // Reminder on the type names below: "LL" = Limelight (the camera), and a
        // "Fiducial" is an AprilTag. So latestResult = everything the camera saw this
        // frame, and each FiducialResult in it is one AprilTag it spotted.
        LLResult latestResult = limelightCamera.getLatestResult();

        // Pull the list of AprilTags out of the camera's result — but only if the
        // camera actually returned something this loop. Otherwise leave the list empty.
        List<LLResultTypes.FiducialResult> detectedTags = null;
        if (latestResult != null) {
            detectedTags = latestResult.getFiducialResults();
        }

        // Show how many tags we can see right now (0 if we have no list).
        long nTagsSeen = 0;
        if (detectedTags != null) {
            nTagsSeen = detectedTags.size();
        }
        telemetryLog.addData("Tags seen", nTagsSeen);

        LLResultTypes.FiducialResult targetTag = null;
        if (latestResult != null && latestResult.isValid() && detectedTags != null) {
            for (LLResultTypes.FiducialResult tag : detectedTags) {
                if (tag.getFiducialId() == TARGET_TAG_ID) { targetTag = tag; break; }
            }
        }

        // ── Our tag is NOT in view right now. We know because the search loop above
        //    never found tag #11, so targetTag is still null. What we do next depends
        //    on whether we have ever seen it: if not, keep scanning for it; if we had
        //    it and just lost sight of it, spin to look again and eventually give up. ──
        if (targetTag == null) {
            if (!bIsTagAcquired) {
                // Never seen the tag yet — spin slowly in place to look for it.
                // setTeleOpDrive(forward, strafe, turn, robotCentric): each of the
                // first three is a motor power from -1.0 to 1.0. forward = drive ahead/
                // back, strafe = slide left/right, turn = rotate. Here only turn is set
                // (0.15), so the robot rotates in place without going anywhere.
                pathFollower.setTeleOpDrive(0.0, 0.0, 0.15, true);
                telemetryLog.debug("Phase", "SCAN – looking for tag " + TARGET_TAG_ID);
            } else {
                // We HAD the tag but just lost sight of it. Spin slowly to try to find
                // it again. If it's still gone after 2 seconds, give up: drive back to
                // where we were when we lost it, and then run the triangle anyway.
                if (nTagLostAtMilliseconds < 0) {
                    // First loop with the tag missing: remember the clock time and where
                    // the robot is right now, so we can measure the 2 seconds and return here.
                    nTagLostAtMilliseconds = System.currentTimeMillis();
                    tagLostPose = pathFollower.getPose();
                }

                long nMillisecondsSinceTagLost =
                        System.currentTimeMillis() - nTagLostAtMilliseconds;
                if (nMillisecondsSinceTagLost > 2000) {
                    pathFollower.setTeleOpDrive(0.0, 0.0, 0.0, true);
                    bIsAprilTagDone = true;
                    bIsReturnNeeded = true;
                    Log.i(TAG, String.format(
                            "Lost tag for >2 s — returning to (%.1f, %.1f) then triangle",
                            tagLostPose.getX(), tagLostPose.getY()));
                    telemetryLog.debug("Phase", "TIMEOUT – returning to lost pose");
                } else {
                    pathFollower.setTeleOpDrive(0.0, 0.0, 0.12, true);
                    telemetryLog.debug("Phase",
                            String.format("REACQUIRE (%.1f s)", nMillisecondsSinceTagLost / 1000.0));
                }
            }
            return;
        }

        // Tag is visible — reset lost timer.
        nTagLostAtMilliseconds = -1L;

        bIsTagAcquired = true;

        // TX: how far left/right the tag is from center, in degrees (0 = straight
        //     ahead, negative = tag is to the left, positive = to the right).
        double fTargetXDegrees    = targetTag.getTargetXDegrees();
        // TA: how much of the camera image the tag fills, as a percent. It grows as
        //     the robot gets closer, so we use it as a rough stand-in for distance.
        double fTargetAreaPercent = targetTag.getTargetArea();

        // Remember the largest TA we've seen so far this run.
        fMaxTargetAreaSeen = Math.max(fMaxTargetAreaSeen, fTargetAreaPercent);

        telemetryLog.addData("TX  (deg)", fTargetXDegrees);
        telemetryLog.addData("TA  (%)",   fTargetAreaPercent);
        telemetryLog.addData("TA goal",   TARGET_AREA_CLOSE);
        telemetryLog.addData("TA max",    fMaxTargetAreaSeen);

        // ── Parked: we're close enough (TA reached the goal) AND aimed at the tag
        //    (TX within 2°). Stop the motors, mark the camera phase finished, and the
        //    next loop() will start the triangle. ──
        if (fTargetAreaPercent >= TARGET_AREA_CLOSE && Math.abs(fTargetXDegrees) <= 2.0) {
            pathFollower.setTeleOpDrive(0.0, 0.0, 0.0, true);
            bIsAprilTagDone = true;
            Log.i(TAG, String.format("PARKED (ta=%.1f%%, tx=%.1f°)",
                    fTargetAreaPercent, fTargetXDegrees));
            telemetryLog.debug("Phase", String.format("PARKED (ta=%.1f%%)", fTargetAreaPercent));
            return;
        }

        // ── PRE-ALIGN: turn in place until the robot is facing the tag (TX within
        //    ±2°) BEFORE it ever drives forward, so it can't approach at an angle.
        //    Once it has faced the tag, bIsAlignedOnce stays true and we skip this. ──
        if (!bIsAlignedOnce) {
            if (Math.abs(fTargetXDegrees) > 2.0) {
                // Turn power proportional to how far off center the tag is: the more
                // off-center (bigger TX), the harder we turn. Negative because a tag
                // to the right (positive TX) needs a right turn (negative turn power).
                double fTurnPower = -fTargetXDegrees * 0.02;
                // Never turn harder than 0.20 (clamp the value into the -0.20..0.20 range).
                fTurnPower = Math.max(-0.20, Math.min(0.20, fTurnPower));
                // ...but always turn at least 0.06, or the robot is too weak to move.
                if (Math.abs(fTurnPower) < 0.06) fTurnPower = Math.copySign(0.06, fTurnPower);
                pathFollower.setTeleOpDrive(0.0, 0.0, fTurnPower, true);
                telemetryLog.debug("Phase", String.format("PRE-ALIGN tx=%.1f°", fTargetXDegrees));
                return;
            }
            bIsAlignedOnce = true;
        }

        // ── SQUARE-UP: slide sideways ("strafe") until the robot is directly in
        //    front of the tag, not off to one side. We ask the camera where the robot
        //    is as seen FROM THE TAG: its side-to-side distance tells us how far off
        //    center we are. We strafe to erase that distance while turning a little to
        //    keep the tag centered in view. Once centered, bIsSquaredUp stays true so
        //    this step doesn't keep re-triggering as we get close. ──
        if (USE_SQUAREUP && !bIsSquaredUp) {
            Pose3D robotPoseInTagFrame = targetTag.getRobotPoseTargetSpace();

            // Pull the robot's position (converted to inches) out of that pose, but
            // only if the camera actually gave us one this loop.
            Position robotPositionInTagFrame = null;
            if (robotPoseInTagFrame != null) {
                robotPositionInTagFrame = robotPoseInTagFrame.getPosition().toUnit(DistanceUnit.INCH);
            }

            if (robotPositionInTagFrame != null) {
                // Camera data is flowing — clear the "waiting for data" stopwatch.
                nSquareUpWaitStartMilliseconds = -1L;

                double fLateralInches = robotPositionInTagFrame.x;   // +x = robot right of center
                double fYawDegrees = robotPoseInTagFrame.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetryLog.addData("Lateral (in)", fLateralInches);
                telemetryLog.addData("Depth Z (in)", robotPositionInTagFrame.z);
                telemetryLog.addData("Yaw (deg)",    fYawDegrees);

                if (Math.abs(fLateralInches) > CENTER_TOLERANCE_INCHES) {
                    // Strafe toward centerline. If it strafes the WRONG way on your
                    // robot, flip the sign of STRAFE_GAIN.
                    double fStrafePower = -fLateralInches * STRAFE_GAIN;
                    fStrafePower = Math.max(-STRAFE_MAX, Math.min(STRAFE_MAX, fStrafePower));
                    if (Math.abs(fStrafePower) < 0.08) fStrafePower = Math.copySign(0.08, fStrafePower);

                    // While sliding sideways, also turn a little so the tag stays
                    // centered in the camera and we don't lose sight of it.
                    double fCenteringTurnPower = -fTargetXDegrees * 0.02;
                    fCenteringTurnPower = Math.max(-0.15, Math.min(0.15, fCenteringTurnPower));
                    pathFollower.setTeleOpDrive(0.0, fStrafePower, fCenteringTurnPower, true);
                    telemetryLog.debug("Phase",
                            String.format("SQUARE-UP lat=%.1fin tx=%.1f°",
                                    fLateralInches, fTargetXDegrees));
                    return;
                }

                bIsSquaredUp = true;   // on the centerline — proceed to approach
            } else {
                // The camera didn't give us a position this time around. Stop moving
                // and wait; we'll try again on the next loop.
                //
                // BUT do not wait forever. If the camera NEVER provides position data
                // (for example, the pipeline was never configured for 3D), this branch
                // would otherwise repeat every loop and the robot would sit frozen here
                // for the rest of the autonomous. So: after 1.5 seconds of waiting,
                // give up on squaring up and fall through to the normal point-and-drive
                // approach, which only needs TX and TA and always works.
                if (nSquareUpWaitStartMilliseconds < 0) {
                    nSquareUpWaitStartMilliseconds = System.currentTimeMillis();
                }

                long nMillisecondsWaitingForPose =
                        System.currentTimeMillis() - nSquareUpWaitStartMilliseconds;
                if (nMillisecondsWaitingForPose > 1500) {
                    bIsSquaredUp = true;   // skip square-up; plain approach still works
                    Log.w(TAG, "Square-up: no camera position data for 1.5 s — skipping square-up");
                    telemetryLog.debug("Phase", "SQUARE-UP skipped (no position data)");
                } else {
                    pathFollower.setTeleOpDrive(0.0, 0.0, 0.0, true);
                    telemetryLog.debug("Phase", "SQUARE-UP (waiting for pose)");
                    return;
                }
            }
        }

        // ── Steering, done every loop while approaching: if the tag has drifted more
        //    than 1.5° off center, add a gentle turn to re-aim at it. This is the same
        //    idea as PRE-ALIGN above, but here it runs at the same time as driving
        //    forward, so the robot curves toward the tag instead of stopping to turn. ──
        double fTurnPower = 0.0;
        if (Math.abs(fTargetXDegrees) > 1.5) {
            fTurnPower = -fTargetXDegrees * 0.02;                 // turn harder the more off-center it is
            fTurnPower = Math.max(-0.20, Math.min(0.20, fTurnPower));  // keep it within -0.20..0.20
            if (Math.abs(fTurnPower) < 0.06) fTurnPower = Math.copySign(0.06, fTurnPower);  // but at least 0.06
        }

        // ── Drive forward toward the tag, but only while we're roughly pointed at it
        //    (TX within 20°). If we're way off to the side, don't drive yet — let the
        //    turn above aim us first, so we don't head off at an angle. ──
        double fDrivePower = 0.0;
        if (fTargetAreaPercent < TARGET_AREA_CLOSE && Math.abs(fTargetXDegrees) < 20.0) {
            // TA grows as we get closer, so (goal - TA) is how much "gap" is left to
            // close. Drive faster when the gap is big, and ease off as it shrinks.
            double fTargetAreaGap = TARGET_AREA_CLOSE - fTargetAreaPercent;
            fDrivePower = fTargetAreaGap * 0.01;
            fDrivePower = Math.min(0.30, fDrivePower);   // never faster than 0.30
            fDrivePower = Math.max(0.10, fDrivePower);   // never slower than 0.10, or it stalls
        }

        pathFollower.setTeleOpDrive(fDrivePower, 0.0, fTurnPower, true);

        // Dashboard label: are we driving forward as well as steering, or only steering?
        String strPhase = "STEER ONLY";
        if (fDrivePower != 0.0) {
            strPhase = "APPROACH+STEER";
        }
        telemetryLog.debug("Phase",
                String.format("%s tx=%.1f° ta=%.1f%%",
                        strPhase, fTargetXDegrees, fTargetAreaPercent));
    }

    /**
     * WHAT: One-time actions the instant the match begins. Resets the elapsed-time
     *       timer that loop() uses to throttle its once-per-second position log, and
     *       (only if RUN_LOG_DEMO is on) prints one line at each Android log level as
     *       a demonstration.
     *
     * CALLED FROM: the FTC SDK automatically (OpMode lifecycle) — runs ONCE when the
     *       driver presses PLAY, after init() and just before the first loop().
     */
    @Override
    public void start() {
        Log.i(TAG, "start()");
        elapsedTimer.resetTimer();
        if (RUN_LOG_DEMO) {
            Log.v(TAG, "VERBOSE"); Log.d(TAG, "DEBUG");
            Log.i(TAG, "INFO");   Log.w(TAG, "WARN"); Log.e(TAG, "ERROR");
        }
    }

    /**
     * WHAT: The main body of the op mode. Each call advances whichever phase is
     *       currently active: first the AprilTag vision phase (delegated to
     *       aprilTagUpdate()); then, if the tag was lost and we timed out, the
     *       return-to-lost-pose phase; then the five-leg triangle. It also publishes
     *       live telemetry every call and logs the robot's position once per second.
     *
     * CALLED FROM: the FTC SDK automatically (OpMode lifecycle) — runs REPEATEDLY,
     *       many times per second, from when PLAY is pressed until the op mode stops.
     */
    @Override
    public void loop() {
        if (!bIsAprilTagDone) {
            // ── VISION PHASE: drive to the AprilTag (all handled by aprilTagUpdate). ──
            pathFollower.update();    // let the follower re-measure where the robot is
            aprilTagUpdate();

        } else if (RUN_PATHS) {

            // ── RETURN PHASE: only runs if we lost the tag and gave up above. ─────
            // Drive back to the exact spot where the tag was lost. Once we get there,
            // the "else" branch below (the triangle) takes over on the following loops.
            if (bIsReturnNeeded && !bIsReturnDone) {
                if (returnPath == null) {
                    // Measure the straight-line distance from where we are now to the
                    // spot we want to return to (fDeltaX/fDeltaY are the X and Y gaps).
                    Pose currentPose = pathFollower.getPose();
                    double fDeltaX = tagLostPose.getX() - currentPose.getX();
                    double fDeltaY = tagLostPose.getY() - currentPose.getY();
                    if (Math.sqrt(fDeltaX * fDeltaX + fDeltaY * fDeltaY) < 2.0) {
                        // Already within 2 inches of the spot — no need to drive; done.
                        bIsReturnDone = true;
                    } else {
                        returnPath = pathFollower.pathBuilder()
                                .addPath(new BezierLine(currentPose, tagLostPose))
                                .setConstantHeadingInterpolation(tagLostPose.getHeading())
                                .build();
                    }
                }
                if (!bIsReturnDone) {
                    pathFollower.update();
                    if (!bIsReturnStarted) {
                        pathFollower.followPath(returnPath, true);
                        bIsReturnStarted = true;
                    }
                    if (!pathFollower.isBusy()) bIsReturnDone = true;
                    telemetryLog.debug("Phase", "RETURNING to lost pose");
                }

            // ── TRIANGLE PHASE ────────────────────────────────────────────
            } else {
                // Runs ONCE, the first time we reach the triangle. During the camera
                // phase the robot spun and drove around by hand, so its own idea of
                // where it sits on the field is now unreliable. We fix that by telling
                // the follower "you are exactly at the start point" — a clean, known
                // Pose — so the pre-built triangle legs drive correctly from here.
                if (!bIsPathPhaseInitialized) {
                    // Stop the by-hand (teleop) driving and clear any leftover motion.
                    pathFollower.breakFollowing();

                    // Which heading do we DECLARE the robot to have as the triangle starts?
                    //  - If the camera phase ran, the robot is parked facing the tag. We
                    //    declare its heading to be π radians (180°); because leg 1 is set
                    //    to hold heading 0, the robot first spins 180° (turning away from
                    //    the tag, toward the field) and then drives the triangle.
                    //  - If the camera phase was compiled OUT (RUN_APRILTAG = false), the
                    //    robot is still sitting at its real start spot facing forward.
                    //    Declaring 180° there would make it needlessly spin around and
                    //    drive the whole triangle mirrored, so we declare heading 0 —
                    //    matching how the plain triangle op modes behave.
                    double fTriangleStartHeadingRadians = 0.0;
                    if (RUN_APRILTAG) {
                        fTriangleStartHeadingRadians = Math.PI;
                    }
                    pathFollower.setPose(new Pose(
                            triangleBelowBottomMiddle.getX(),
                            triangleBelowBottomMiddle.getY(),
                            fTriangleStartHeadingRadians));
                    bIsPathPhaseInitialized = true;
                    Log.i(TAG, "Triangle: reset frame (heading=180°), starting legs");
                }

                // Drive the triangle one leg at a time. isBusy() is true while the
                // follower is still driving the current leg; when it turns false, that
                // leg is finished and we hand over the next one. Once the last leg
                // (leg 5) has finished, we stop.
                if (bIsLeg5Started && !pathFollower.isBusy()) {
                    pathFollower.breakFollowing();   // whole triangle done — stop driving
                } else {
                    pathFollower.update();

                    // If the robot physically jams partway through a leg, hand it the
                    // current leg again so it re-tries instead of sitting there stuck.
                    if (pathFollower.isRobotStuck() && activeLeg != null) {
                        pathFollower.followPath(activeLeg, true);
                    }

                    // Start leg 1 right away; after that, each "if" fires the next leg
                    // the moment the previous one finishes (isBusy() becomes false).
                    if (!bIsLeg1Started) {
                        activeLeg = leg1StartToTop; pathFollower.followPath(leg1StartToTop, true); bIsLeg1Started = true;
                    }
                    if (!pathFollower.isBusy() && bIsLeg1Started && !bIsLeg2Started) {
                        activeLeg = leg2TopToLeft; pathFollower.followPath(leg2TopToLeft, true); bIsLeg2Started = true;
                    }
                    if (!pathFollower.isBusy() && bIsLeg2Started && !bIsLeg3Started) {
                        activeLeg = leg3LeftToRight; pathFollower.followPath(leg3LeftToRight, true); bIsLeg3Started = true;
                    }
                    if (!pathFollower.isBusy() && bIsLeg3Started && !bIsLeg4Started) {
                        activeLeg = leg4RightToTop; pathFollower.followPath(leg4RightToTop, true); bIsLeg4Started = true;
                    }
                    if (!pathFollower.isBusy() && bIsLeg4Started && !bIsLeg5Started) {
                        activeLeg = leg5TopToStart; pathFollower.followPath(leg5TopToStart, true); bIsLeg5Started = true;
                    }
                }
            }
        }

        // ── Telemetry ─────────────────────────────────────────────────────
        telemetryLog.addData("X", pathFollower.getPose().getX());
        telemetryLog.addData("Y", pathFollower.getPose().getY());
        telemetryLog.debug("Busy", String.valueOf(pathFollower.isBusy()));
        telemetryLog.update(telemetry);

        if (elapsedTimer.getElapsedTimeSeconds() - fLastUpdateTimeSeconds >= 1.0) {
            fLastUpdateTimeSeconds = elapsedTimer.getElapsedTimeSeconds();
            Log.d(TAG, String.format("@%.0fs  x=%.1f y=%.1f busy=%b",
                    elapsedTimer.getElapsedTimeSeconds(),
                    pathFollower.getPose().getX(), pathFollower.getPose().getY(),
                    pathFollower.isBusy()));
        }
    }

    /**
     * WHAT: One-time cleanup when the op mode ends. Flushes the TelemetryLog to a
     *       file so the run's log survives after the op mode exits.
     *
     * CALLED FROM: the FTC SDK automatically (OpMode lifecycle) — runs ONCE when the
     *       driver presses STOP, or when the autonomous period's time runs out.
     */
    @Override
    public void stop() {
        telemetryLog.save();
    }
}

package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * SampleAutoPathingTwoLegs
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   A two-segment Pedro Pathing autonomous. On play the robot drives:
 *     leg1: start  -> top
 *     leg2: top    -> left
 *   then stops cleanly.
 *
 * WHAT IT'S FOR:
 *   The next step up from the single-line test — confirms the robot can chain two
 *   paths back-to-back (starting leg2 only after leg1 finishes).
 *
 * HARDWARE REQUIRED (names must match the Driver Hub robot config):
 *   - "front_left_motor", "front_right_motor", "back_left_motor", "back_right_motor"
 *   - "pinpoint" (goBILDA Pinpoint odometry computer)
 * =====================================================================================
 */

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Drive Forward - Two Legs Test")
public class SampleAutoPathingTwoLegs extends OpMode {

    // Drives the robot and tracks its position on the field.
    private Follower follower;

    // Field positions (in inches) that define the two legs.
    private final Pose startPose = new Pose(71, 62, 0);
    private final Pose topPose   = new Pose(131, 62, 0);
    private final Pose leftPose  = new Pose(79, 92, 0);

    // The two path segments and flags so each starts exactly once.
    private PathChain leg1;
    private PathChain leg2;
    private boolean leg1Started = false;
    private boolean leg2Started = false;

    /** Runs once on INIT: build the follower and both legs. */
    @Override
    public void init() {
        // Create the follower from our tuned Constants and cap speed at 75%.
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.75);

        // leg1: straight line start -> top.
        leg1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, topPose))
                .setConstantHeadingInterpolation(0)
                .build();

        // leg2: straight line top -> left.
        leg2 = follower.pathBuilder()
                .addPath(new BezierLine(topPose, leftPose))
                .setConstantHeadingInterpolation(0)
                .build();

        // Tell the follower where the robot physically starts.
        follower.setPose(startPose);
    }

    /** Runs repeatedly on PLAY: fire each leg in order, then stop. */
    @Override
    public void loop() {
        // Once leg2 has run and the follower is idle, cut power so it doesn't
        // keep correcting (which would make it spin in place).
        if (leg2Started && !follower.isBusy()) {
            follower.breakFollowing();
            return;
        }

        // Update odometry and motor outputs every loop.
        follower.update();

        // Start leg1 once.
        if (!leg1Started) {
            follower.followPath(leg1, true);
            leg1Started = true;
        }
        // Start leg2 once, only after leg1 has finished (follower no longer busy).
        if (!follower.isBusy() && leg1Started && !leg2Started) {
            follower.followPath(leg2, true);
            leg2Started = true;
        }
    }
}

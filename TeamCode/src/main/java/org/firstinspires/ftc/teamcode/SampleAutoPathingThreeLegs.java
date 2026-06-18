package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * SampleAutoPathingThreeLegs
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   A three-segment Pedro Pathing autonomous. On play the robot drives:
 *     leg1: start -> top
 *     leg2: top   -> left
 *     leg3: left  -> right
 *   then stops cleanly.
 *
 * WHAT IT'S FOR:
 *   Builds on the two-leg test by chaining a third path, on the way to the full
 *   five-leg triangle.
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

@Autonomous(name = "Drive Forward - Three Legs Test")
public class SampleAutoPathingThreeLegs extends OpMode {

    // Drives the robot and tracks its position on the field.
    private Follower follower;

    // Field positions (in inches) that define the three legs.
    private final Pose startPose = new Pose(71, 62, 0);
    private final Pose topPose   = new Pose(131, 62, 0);
    private final Pose leftPose  = new Pose(79, 92, 0);
    private final Pose rightPose = new Pose(79, 32, 0);

    // The three path segments and flags so each starts exactly once.
    private PathChain leg1;
    private PathChain leg2;
    private PathChain leg3;
    private boolean leg1Started = false;
    private boolean leg2Started = false;
    private boolean leg3Started = false;

    /** Runs once on INIT: build the follower and all three legs. */
    @Override
    public void init() {
        // Create the follower from our tuned Constants and cap speed at 75%.
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.75);

        // leg1: start -> top
        leg1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, topPose))
                .setConstantHeadingInterpolation(0)
                .build();

        // leg2: top -> left
        leg2 = follower.pathBuilder()
                .addPath(new BezierLine(topPose, leftPose))
                .setConstantHeadingInterpolation(0)
                .build();

        // leg3: left -> right
        leg3 = follower.pathBuilder()
                .addPath(new BezierLine(leftPose, rightPose))
                .setConstantHeadingInterpolation(0)
                .build();

        // Tell the follower where the robot physically starts.
        follower.setPose(startPose);
    }

    /** Runs repeatedly on PLAY: fire each leg in order, then stop. */
    @Override
    public void loop() {
        // Once the last leg has run and the follower is idle, cut power so it
        // doesn't keep correcting (which would make it spin in place).
        if (leg3Started && !follower.isBusy()) {
            follower.breakFollowing();
            return;
        }

        // Update odometry and motor outputs every loop.
        follower.update();

        // Fire each leg once, in order — each one starts only after the previous
        // finishes (follower no longer busy).
        if (!leg1Started) {
            follower.followPath(leg1, true);
            leg1Started = true;
        }
        if (!follower.isBusy() && leg1Started && !leg2Started) {
            follower.followPath(leg2, true);
            leg2Started = true;
        }
        if (!follower.isBusy() && leg2Started && !leg3Started) {
            follower.followPath(leg3, true);
            leg3Started = true;
        }
    }
}

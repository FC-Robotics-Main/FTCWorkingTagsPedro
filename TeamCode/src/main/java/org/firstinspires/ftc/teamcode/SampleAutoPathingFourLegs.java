package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * SampleAutoPathingFourLegs
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   A four-segment Pedro Pathing autonomous. On play the robot drives:
 *     leg1: start -> top
 *     leg2: top   -> left
 *     leg3: left  -> right
 *     leg4: right -> top
 *   then stops cleanly. Also includes basic "stuck" recovery: if the robot stops
 *   making progress mid-path, it re-issues the current leg.
 *
 * WHAT IT'S FOR:
 *   Builds on the three-leg test, adding a fourth path plus stuck-detection, on the
 *   way to the full five-leg triangle.
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

@Autonomous(name = "Drive Forward - Four Legs Test")
public class SampleAutoPathingFourLegs extends OpMode {

    // Drives the robot and tracks its position on the field.
    private Follower follower;

    // Field positions (in inches) that define the four legs.
    private final Pose startPose = new Pose(71, 62, 0);
    private final Pose topPose   = new Pose(131, 62, 0);
    private final Pose leftPose  = new Pose(79, 92, 0);
    private final Pose rightPose = new Pose(79, 32, 0);

    // The four path segments and flags so each starts exactly once.
    private PathChain leg1;
    private PathChain leg2;
    private PathChain leg3;
    private PathChain leg4;
    private boolean leg1Started = false;
    private boolean leg2Started = false;
    private boolean leg3Started = false;
    private boolean leg4Started = false;

    // The leg currently being followed — used by the stuck-recovery logic.
    private PathChain activeLeg = null;

    /** Runs once on INIT: build the follower and all four legs. */
    @Override
    public void init() {
        // Create the follower from our tuned Constants and cap speed at 50%.
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.5);

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

        // leg4: right -> top
        leg4 = follower.pathBuilder()
                .addPath(new BezierLine(rightPose, topPose))
                .setConstantHeadingInterpolation(0)
                .build();

        // Tell the follower where the robot physically starts.
        follower.setPose(startPose);
    }

    /** Runs repeatedly on PLAY: fire each leg in order, recover if stuck, then stop. */
    @Override
    public void loop() {
        // Once the last leg has run and the follower is idle, cut power so it
        // doesn't keep correcting (which would make it spin in place).
        if (leg4Started && !follower.isBusy()) {
            follower.breakFollowing();
            return;
        }

        // Update odometry and motor outputs every loop.
        follower.update();

        // If the robot gets physically stuck mid-path, re-issue the current leg.
        if (follower.isRobotStuck() && activeLeg != null) {
            follower.followPath(activeLeg, true);
        }

        // Fire each leg once, in order — each starts only after the previous finishes.
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
    }
}

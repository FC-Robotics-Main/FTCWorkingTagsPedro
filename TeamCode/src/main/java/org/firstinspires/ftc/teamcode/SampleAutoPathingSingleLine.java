package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * SampleAutoPathingSingleLine
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   The simplest possible Pedro Pathing autonomous: on play, the robot drives ONE
 *   straight line from the start position to the "top" position, then stops.
 *
 * WHAT IT'S FOR:
 *   A bare-bones test to confirm the drivetrain, odometry, and Pedro Pathing follower
 *   are all working before adding more complex multi-leg routes.
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

@Autonomous(name = "Drive Forward - Single Line Test")
public class SampleAutoPathingSingleLine extends OpMode {

    // Drives the robot and tracks its position on the field.
    private Follower follower;

    // Field positions (in inches): where we start and where the single line ends.
    private final Pose startPose = new Pose(71, 62, 0);
    private final Pose topPose = new Pose(131, 62, 0);

    // The one path segment, and a flag so we only start it once.
    private PathChain leg1;
    private boolean leg1Started = false;

    /** Runs once on INIT: build the follower and the single path. */
    @Override
    public void init() {
        // Create the follower from our tuned Constants and cap speed at 75%.
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.75);

        // Build a straight line from start to top, holding heading 0 the whole way.
        leg1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, topPose))
                .setConstantHeadingInterpolation(0)
                .build();

        // Tell the follower where the robot physically starts.
        follower.setPose(startPose);
    }

    /** Runs repeatedly on PLAY: start the line once, then let the follower drive it. */
    @Override
    public void loop() {
        // Update odometry and motor outputs every loop.
        follower.update();

        // Fire the path a single time.
        if (!leg1Started) {
            follower.followPath(leg1, true);
            leg1Started = true;
        }
    }
}

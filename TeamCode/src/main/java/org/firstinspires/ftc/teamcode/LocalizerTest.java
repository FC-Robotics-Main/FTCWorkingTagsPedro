package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * LocalizerTest
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   A diagnostic TeleOp that does NOT drive the robot. It just prints the follower's
 *   estimated position (X, Y) and heading. You push the robot around by hand and watch
 *   the numbers to confirm the odometry / Pinpoint is reading correctly.
 *
 * WHAT IT'S FOR:
 *   Verifying encoder directions and axis orientation after moving/remounting the
 *   Pinpoint or odometry pods. Pedro Pathing's convention is:
 *     - Push the robot FORWARD       -> X should INCREASE
 *     - Push the robot LEFT          -> Y should INCREASE
 *     - Rotate COUNTERCLOCKWISE (left) -> Heading should INCREASE
 *   If any axis goes the wrong way, fix the encoder directions (or swap the pod
 *   cables) in pedroPathing/Constants.java.
 *
 * HARDWARE REQUIRED (names must match the Driver Hub robot config):
 *   - "pinpoint" (goBILDA Pinpoint odometry computer)
 *   - the four drive motors (the follower expects them even though we don't drive)
 * =====================================================================================
 */

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Localizer Test")
public class LocalizerTest extends OpMode {

    // Tracks the robot's position; we read from it but never command it to drive.
    private Follower follower;

    /** Runs once on INIT: build the follower and zero the pose. */
    @Override
    public void init() {
        // Build the follower and reset its position to the origin so the numbers
        // are easy to read as you push the robot.
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        // On-screen reminder of the expected (correct) directions.
        telemetry.addLine("Push robot around to verify odometry directions:");
        telemetry.addLine("  Forward         -> X should increase");
        telemetry.addLine("  Left            -> Y should increase");
        telemetry.addLine("  Rotate CCW (left) -> Heading should increase");
        telemetry.update();
    }

    /** Runs repeatedly on PLAY: refresh odometry and print the current pose. */
    @Override
    public void loop() {
        // Update the position estimate from the Pinpoint.
        follower.update();

        // Read and display the current pose.
        Pose pose = follower.getPose();
        telemetry.addData("X (forward)", String.format("%.2f", pose.getX()));
        telemetry.addData("Y (left)", String.format("%.2f", pose.getY()));
        telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(pose.getHeading())));
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Drive Forward - Four Legs Test")
public class SampleAutoPathingFourLegs extends OpMode {

    private Follower follower;

    private final Pose startPose = new Pose(71, 62, 0);
    private final Pose topPose   = new Pose(131, 62, 0);
    private final Pose leftPose  = new Pose(79, 92, 0);
    private final Pose rightPose = new Pose(79, 32, 0);

    private PathChain leg1;
    private PathChain leg2;
    private PathChain leg3;
    private PathChain leg4;
    private boolean leg1Started = false;
    private boolean leg2Started = false;
    private boolean leg3Started = false;
    private boolean leg4Started = false;

    private PathChain activeLeg = null;

    @Override
    public void init() {
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

        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        if (leg4Started && !follower.isBusy()) {
            follower.breakFollowing();
            return;
        }

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
    }
}

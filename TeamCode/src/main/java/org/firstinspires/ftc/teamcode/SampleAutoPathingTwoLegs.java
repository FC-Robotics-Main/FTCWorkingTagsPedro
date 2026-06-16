package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Drive Forward - Two Legs Test")
public class SampleAutoPathingTwoLegs extends OpMode {

    private Follower follower;

    private final Pose startPose = new Pose(71, 62, 0);
    private final Pose topPose   = new Pose(131, 62, 0);
    private final Pose leftPose  = new Pose(79, 92, 0);

    private PathChain leg1;
    private PathChain leg2;
    private boolean leg1Started = false;
    private boolean leg2Started = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.75);

        leg1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, topPose))
                .setConstantHeadingInterpolation(0)
                .build();

        leg2 = follower.pathBuilder()
                .addPath(new BezierLine(topPose, leftPose))
                .setConstantHeadingInterpolation(0)
                .build();

        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        if (leg2Started && !follower.isBusy()) {
            follower.breakFollowing();
            return;
        }

        follower.update();

        if (!leg1Started) {
            follower.followPath(leg1, true);
            leg1Started = true;
        }
        if (!follower.isBusy() && leg1Started && !leg2Started) {
            follower.followPath(leg2, true);
            leg2Started = true;
        }
    }
}

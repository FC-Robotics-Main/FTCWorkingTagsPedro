package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Drive Forward - Single Line Test")
public class SampleAutoPathingSingleLine extends OpMode {

    private Follower follower;

    private final Pose startPose = new Pose(71, 62, 0);
    private final Pose topPose = new Pose(131, 62, 0);

    private PathChain leg1;
    private boolean leg1Started = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.75);

        leg1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, topPose))
                .setConstantHeadingInterpolation(0)
                .build();

        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();

        if (!leg1Started) {
            follower.followPath(leg1, true);
            leg1Started = true;
        }
    }
}

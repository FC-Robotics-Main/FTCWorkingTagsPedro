package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Localizer Test")
public class LocalizerTest extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        telemetry.addLine("Push robot around to verify encoder directions:");
        telemetry.addLine("  Forward -> Y should increase");
        telemetry.addLine("  Strafe right -> X should increase");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.getPose();
        telemetry.addData("X (strafe)", String.format("%.2f", pose.getX()));
        telemetry.addData("Y (forward)", String.format("%.2f", pose.getY()));
        telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(pose.getHeading())));
        telemetry.update();
    }
}

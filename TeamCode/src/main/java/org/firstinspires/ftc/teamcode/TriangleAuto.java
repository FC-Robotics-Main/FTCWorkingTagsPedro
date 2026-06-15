package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drives a closed equilateral triangle (60in sides): A -> B -> C -> A.
 */
@Autonomous(name = "Equilateral Triangle Auto")
public class TriangleAuto extends OpMode {
    private static final String TAG = "TriangleAuto";
    private static final double SIDE = 60.0;

    private final Pose vertexA = new Pose(71, 62, 0);
    private final Pose vertexB = new Pose(vertexA.getX() + SIDE, vertexA.getY(), 0);
    private final Pose vertexC = new Pose(
            vertexA.getX() + SIDE / 2.0,
            vertexA.getY() + SIDE * Math.sqrt(3) / 2.0,
            0);

    private Follower follower;
    private TelemetryLog telemetryLog;

    private PathChain legAB;
    private PathChain legBC;
    private PathChain legCA;

    private boolean legBCStarted = false;
    private boolean legCAStarted = false;

    @Override
    public void init() {
        telemetryLog = new TelemetryLog(PanelsTelemetry.INSTANCE.getTelemetry(), "TriangleAuto");
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.75);

        legAB = follower.pathBuilder()
                .addPath(new BezierLine(vertexA, vertexB))
                .setConstantHeadingInterpolation(0)
                .build();
        legBC = follower.pathBuilder()
                .addPath(new BezierLine(vertexB, vertexC))
                .setConstantHeadingInterpolation(0)
                .build();
        legCA = follower.pathBuilder()
                .addPath(new BezierLine(vertexC, vertexA))
                .setConstantHeadingInterpolation(0)
                .build();

        follower.setPose(vertexA);
        Log.i(TAG, "init complete: A=" + vertexA + " B=" + vertexB + " C=" + vertexC);
    }

    @Override
    public void start() {
        follower.followPath(legAB, true);
        Log.i(TAG, "start: leg A->B");
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy() && !legBCStarted) {
            follower.followPath(legBC, true);
            legBCStarted = true;
            Log.i(TAG, "leg B->C");
        } else if (!follower.isBusy() && legBCStarted && !legCAStarted) {
            follower.followPath(legCA, true);
            legCAStarted = true;
            Log.i(TAG, "leg C->A");
        }

        telemetryLog.addData("X", follower.getPose().getX());
        telemetryLog.addData("Y", follower.getPose().getY());
        telemetryLog.addData("Heading", follower.getPose().getHeading());
        telemetryLog.debug("Busy", String.valueOf(follower.isBusy()));
        telemetryLog.update(telemetry);
    }

    @Override
    public void stop() {
        telemetryLog.save();
    }
}

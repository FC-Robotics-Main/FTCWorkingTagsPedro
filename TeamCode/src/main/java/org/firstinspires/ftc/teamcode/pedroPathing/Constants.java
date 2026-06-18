package org.firstinspires.ftc.teamcode.pedroPathing;

/* =====================================================================================
 * Constants
 * -------------------------------------------------------------------------------------
 * WHAT THIS CLASS DOES:
 *   Central configuration file for the Pedro Pathing follower. All tuning values
 *   live here as static fields so they can be changed in one place and picked up
 *   everywhere. Also provides the createFollower() factory used by every autonomous.
 *
 * KEY VALUES (change here, nowhere else):
 *   followerConstants — mass, zero-power deceleration, PIDF gains for translation,
 *                       heading, and drive.
 *   driveConstants    — motor hardware names, directions, and calibrated max velocities.
 *   localizerConstants — Pinpoint pod offsets (mm from robot center), encoder
 *                        directions, and hardware map name.
 *   pathConstraints   — global speed limits passed to every PathChain.
 *
 * AFTER MOVING HARDWARE:
 *   - If you remount the odometry pods, re-measure and update forwardPodY / strafePodX
 *     (distance in mm from the robot center to each pod).
 *   - If the robot goes in the wrong direction, flip forwardEncoderDirection or
 *     strafeEncoderDirection here (or swap the physical pod cables on the Pinpoint).
 *   - Run LocalizerTest to verify: push forward->X up, push left->Y up, rotate CCW->heading up.
 * =====================================================================================
 */

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.3)
            .forwardZeroPowerAcceleration(-33.228152025062755)
            .lateralZeroPowerAcceleration(-69.57792124157147)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(0.85, 0, 0, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.4, 0, 0, 0.6, 0.025))
            .centripetalScaling(0.0005)

            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_motor")
            .rightRearMotorName("back_right_motor")
            .leftRearMotorName("back_left_motor")
            .leftFrontMotorName("front_left_motor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(60.50015667292077)
            .yVelocity(43.335864450049215);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7.5)
            .strafePodX(7.84)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}

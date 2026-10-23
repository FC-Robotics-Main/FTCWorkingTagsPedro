package org.firstinspires.ftc.teamcode.pedroPathing;

/* =====================================================================================
 * Constants  —  FULL DEFAULT (new robot)
 * -------------------------------------------------------------------------------------
 * Everything is back to Pedro Pathing quickstart / library defaults. Nothing here
 * reflects the real robot yet.
 *
 * BEFORE TUNING — fill in the hardware:
 *   a. Motor names below must match the names in the Driver Hub robot configuration.
 *   b. Motor directions: run the robot and flip any wheel that spins backward.
 *   c. Pinpoint: hardwareMapName must match the config; measure pod offsets from
 *      robot center (forwardPodY = left/right offset of the forward pod, + is left;
 *      strafePodX = forward/back offset of the strafe pod, + is forward).
 *   d. Mass: weigh the robot and add .mass(kg) to followerConstants.
 *
 * RETUNE ORDER (paste each result back in here):
 *   1. LocalizationTest   — push forward -> X up, push left -> Y up, rotate CCW -> heading up.
 *   2. ForwardVelocityTuner          -> .xVelocity(...)                  in driveConstants
 *   3. LateralVelocityTuner          -> .yVelocity(...)                  in driveConstants
 *   4. ForwardZeroPowerAccelTuner    -> .forwardZeroPowerAcceleration()  in followerConstants
 *   5. LateralZeroPowerAccelTuner    -> .lateralZeroPowerAcceleration()  in followerConstants
 *   6. Translational PIDF            -> .translationalPIDFCoefficients()
 *   7. Heading PIDF                  -> .headingPIDFCoefficients()
 *   8. Drive PIDF                    -> .drivePIDFCoefficients()
 *   9. Centripetal                   -> .centripetalScaling()
 *  10. Verify with Line / Triangle / Circle tests.
 * =====================================================================================
 */

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

    // Mass: 35.4 lb = 16.06 kg
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(16.06)
            .forwardZeroPowerAcceleration(-19.91)  // avg of -20.40, -16.68, -22.65
            .lateralZeroPowerAcceleration(-63.72); // avg of -67.98, -76.05, -58.35, -52.50 (dropped outliers -112.34, -30.98)

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FRM")
            .rightRearMotorName("BRM")
            .leftRearMotorName("BLM")
            .leftFrontMotorName("FLM")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(56.67)    // Forward Velocity Tuner avg of 53, 58, 59
            .yVelocity(41.17);   // Lateral Velocity Tuner avg of 41.89, 41.38, 40.24

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-0.3125) // measured: forward pod 5/16" right of center (inches, left +, right -)
            .strafePodX(-1.625)  // measured: strafe pod 1 10/16" behind center (inches, front +)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
package org.firstinspires.ftc.teamcode.pedroPathing;

/* =====================================================================================
 * Constants  —  TUNING RESET (2026-10-29)
 * -------------------------------------------------------------------------------------
 * All tuned values cleared. Library defaults are in effect for velocities, zero-power
 * accelerations, PIDFs, and centripetal scaling.
 *
 * KEPT (hardware/measured, not tuned):
 *   - Mass 16.06 kg (35.4 lb)
 *   - Motor names + directions (FLM/FRM/BLM/BRM)
 *   - Pinpoint name, pod type, encoder directions (verified: forward -> X up, left -> Y up)
 *   - Pod offsets (measured from center of rotation)
 *
 * PRE-CHECK: pods clean, battery >= 13.0 V, 4-tile clear space.
 * Tuning OpMode start pose is (72, 72) — judge changes from 72, not from 0.
 *
 * TUNE IN THIS ORDER (paste each result back in here):
 *   1. LocalizationTest   — fwd 48" -> X ~120, left 48" -> Y ~120, 5 gamepad spins stay within 1"
 *   2. ForwardVelocityTuner          -> .xVelocity(...)                  in driveConstants
 *   3. LateralVelocityTuner          -> .yVelocity(...)                  in driveConstants
 *   4. ForwardZeroPowerAccelTuner    -> .forwardZeroPowerAcceleration()  in followerConstants
 *   5. LateralZeroPowerAccelTuner    -> .lateralZeroPowerAcceleration()  in followerConstants
 *      (run each 3x, drop outliers, average)
 *   6. Translational PIDF            -> .translationalPIDFCoefficients()
 *   7. Heading PIDF                  -> .headingPIDFCoefficients()
 *   8. Drive PIDF + ZPAM (3rd value in PathConstraints, target 2-8) -> .drivePIDFCoefficients()
 *   9. Centripetal                   -> .centripetalScaling()
 *  10. Verify with Line / Triangle / Circle tests.
 * Panels edits are NOT saved to code — copy values here after each tuner.
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

// Uncomment when you reach the PIDF steps:
// import com.pedropathing.control.PIDFCoefficients;
// import com.pedropathing.control.FilteredPIDFCoefficients;

public class Constants {

    // Mass: 35.4 lb = 16.06 kg
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(16.06);
    // .forwardZeroPowerAcceleration(  )   // step 4
    // .lateralZeroPowerAcceleration(  )   // step 5
    // .translationalPIDFCoefficients(new PIDFCoefficients(P, I, D, F))        // step 6
    // .headingPIDFCoefficients(new PIDFCoefficients(P, I, D, F))              // step 7
    // .drivePIDFCoefficients(new FilteredPIDFCoefficients(P, I, D, T, F))     // step 8
    // .centripetalScaling(  )             // step 9

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FRM")
            .rightRearMotorName("BRM")
            .leftRearMotorName("BLM")
            .leftFrontMotorName("FLM")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    // .xVelocity(  )   // step 2
    // .yVelocity(  )   // step 3

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-0.3125) // forward pod 5/16" right of center (inches, left +, right -)
            .strafePodX(-1.625)   // strafe pod 1 10/16" behind center (inches, front +)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // 3rd value = zero power acceleration multiplier (ZPAM): default 3, tune 2-8 in step 8
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 3, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
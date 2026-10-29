package org.firstinspires.ftc.teamcode.pedroPathing;

/* =====================================================================================
 * Constants
 * LAST UPDATED: 2026-10-29 14:47
 *   - Drive P 0.05 and 0.075 had NO effect on the backward-stop offset -> drive PIDF back to
 *     library default (0.025, 0, 0.00001, 0.6, 0.01). Backward Line Test stop ~5" past start,
 *     repeatable; forward stop exact. Parked for now; check in real autos (end-of-path hold).
 * PREVIOUS: 2026-10-29 14:45
 *   - Drive P 0.05: backward stop still ~5" past start (repeatable) -> drive P 0.075. Next: 0.1.
 * PREVIOUS: 2026-10-29 14:38
 *   - -20 gave no further improvement over -24 -> forwardZeroPowerAcceleration back to -24.
 *   - Step 8: drive PIDF set explicitly. Library default (0.025, 0, 0.00001, 0.6, 0.01);
 *     P raised 0.025 -> 0.05 for stronger end-of-path braking. Next if needed: 0.075, 0.1.
 * PREVIOUS: 2026-10-29 14:30
 *   - -24 reduced return-leg overshoot but not eliminated -> forwardZeroPowerAcceleration -20 (test).
 *     Next if needed: -17.
 * PREVIOUS: 2026-10-29 14:27
 *   - Pinpoint remounted flat (sticker up), pods in correct X/Y ports. Localization + spin test pass.
 *   - Line Test: overshoots ~5" past start on return leg (Panels agrees) -> braking too late.
 *     forwardZeroPowerAcceleration -28.303 -> -24 (test).
 * PREVIOUS: 2026-10-29 14:04
 *   Based on checked-in file. Motor AND encoder directions = original checked-in values
 *   (motors L=REVERSE R=FORWARD, forward encoder REVERSED, strafe encoder FORWARD).
 *   Verified with a confirmed deploy (maxPower 0.3 test): flipped encoders read backwards,
 *   so the original directions are correct. Earlier "no change" results were deploys
 *   that never reached the hub (Apply Changes). Use Clean Project + green Run only.
 *   - Added today's measured values: mass, pod offsets, x/y velocity, zero-power accels
 *   - maxPower back to 1
 * -------------------------------------------------------------------------------------
 * PRE-CHECK: pods clean, battery >= 13.0 V, 4-tile clear space.
 * Tuning OpMode start pose is (72, 72) — judge changes from 72, not from 0.
 *
 * VERIFY AFTER DEPLOY (Localization Test, gamepad):
 *   stick forward -> X up | stick left -> Y up | turn CCW -> heading up
 *
 * TUNING STATUS:
 *   1. LocalizationTest                 -> verify directions (above)
 *   2. ForwardVelocityTuner   DONE      -> .xVelocity(62.143)
 *   3. LateralVelocityTuner   DONE      -> .yVelocity(48.508)
 *   4. ForwardZeroPowerAccel  DONE      -> .forwardZeroPowerAcceleration(-28.303)
 *   5. LateralZeroPowerAccel  DONE      -> .lateralZeroPowerAcceleration(-55.496)
 *   6. Translational PIDF     DONE      -> library default, tested good
 *   7. Heading PIDF           DONE      -> library default, tested good
 *   8. Drive PIDF + ZPAM (3rd value in PathConstraints, target 2-8) -> .drivePIDFCoefficients()
 *   9. Centripetal                      -> .centripetalScaling()
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

import com.pedropathing.control.FilteredPIDFCoefficients;

public class Constants {

    // Mass: 35.8 lb = 16.24 kg
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(16.24)
            .forwardZeroPowerAcceleration(-24)      // tuner avg -28.303; -24 helped, -20 no better (14:38)
            .lateralZeroPowerAcceleration(-55.496)  // avg of 5 runs (-47.3 to -63.4)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01)); // P, I, D, T, F — library default (14:47)
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
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(62.143)  // avg of 62.169, 62.185, 62.075
            .yVelocity(48.508); // avg of 48.898, 48.194, 48.433

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-0.177) // forward pod 4.5 mm right of center (inches, left +, right -)
            .strafePodX(-1.831)  // strafe pod 46.5 mm behind center (inches, front +, back -)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)  // original, confirmed 14:04
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);   // original, confirmed 14:04

    // 3rd value = zero power acceleration multiplier (ZPAM): tune 2-8 in step 8
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 3, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
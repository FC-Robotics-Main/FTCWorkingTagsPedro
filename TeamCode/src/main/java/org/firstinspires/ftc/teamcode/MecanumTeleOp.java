package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drive TeleOp.
 *
 * Hardware config names: "FL", "FR", "BL", "BR", "imu"
 *
 * Controls (gamepad1):
 *   Left stick   - drive (forward/back, strafe)
 *   Right stick X - turn
 *   Right bumper - slow mode (hold)
 *   A            - toggle field-centric / robot-centric
 *   Back         - reset IMU heading (field-centric "forward")
 */
@TeleOp(name = "Mecanum TeleOp", group = "Drive")
public class MecanumTeleOp extends LinearOpMode {

    private static final double SLOW_SCALE = 0.35;
    private static final double STRAFE_CORRECTION = 1.1; // counteracts imperfect strafing

    private DcMotor fl, fr, bl, br;
    private IMU imu;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");

        // Left side reversed so +power drives every wheel forward.
        // If the robot spins or drives backward on "forward", flip these.
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Set these to match how the Control Hub is physically mounted.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        boolean fieldCentric = false;
        boolean lastA = false;

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Toggle drive mode on A press
            if (gamepad1.a && !lastA) fieldCentric = !fieldCentric;
            lastA = gamepad1.a;

            if (gamepad1.back) imu.resetYaw();

            double y  = -gamepad1.left_stick_y;                    // forward is +
            double x  =  gamepad1.left_stick_x * STRAFE_CORRECTION; // right is +
            double rx =  gamepad1.right_stick_x;                   // clockwise is +

            if (fieldCentric) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
                x = rotX;
                y = rotY;
            }

            // Normalize so no wheel exceeds 1.0 while keeping ratios
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double flPower = (y + x + rx) / denom;
            double blPower = (y - x + rx) / denom;
            double frPower = (y - x - rx) / denom;
            double brPower = (y + x - rx) / denom;

            double scale = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            fl.setPower(flPower * scale);
            bl.setPower(blPower * scale);
            fr.setPower(frPower * scale);
            br.setPower(brPower * scale);

            telemetry.addData("Mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addData("Slow", gamepad1.right_bumper);
            telemetry.addData("FL/FR", "%.2f / %.2f", flPower * scale, frPower * scale);
            telemetry.addData("BL/BR", "%.2f / %.2f", blPower * scale, brPower * scale);
            telemetry.update();
        }
    }
}
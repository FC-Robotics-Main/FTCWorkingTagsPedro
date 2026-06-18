package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * StarterBotTeleop
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   Basic mecanum-drive TeleOp. Controls:
 *     Left stick Y  — forward / backward
 *     Left stick X  — strafe left / right
 *     Right stick X — rotate
 *     Right bumper  — hold for half-speed precision mode
 *
 * WARNING — DIFFERENT MOTOR NAMES:
 *   This file uses "left_motor_back", "left_motor_front", "right_motor_back",
 *   "right_motor_front" — NOT the same names as the rest of the team's code
 *   ("front_left_motor" etc.). If your Driver Hub config uses the other names,
 *   this OpMode will crash on init. Either rename the motors in the config or
 *   update the hardwareMap.get() calls below to match.
 *
 * HARDWARE REQUIRED (names must match the Driver Hub robot config):
 *   - "left_motor_back", "left_motor_front", "right_motor_back", "right_motor_front"
 * =====================================================================================
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Mecanum Drive TeleOp", group = "TeleOp")
public class StarterBotTeleop extends LinearOpMode {

    // Declare motors
    private DcMotor leftMotorBack;
    private DcMotor leftMotorFront;
    private DcMotor rightMotorBack;
    private DcMotor rightMotorFront;

    // Speed multiplier — tune max speed (0.0 to 1.0)
    private static final double SPEED_MULTIPLIER = 0.8;

    @Override
    public void runOpMode() {

        // Initialize motors
        // Motor port 0 = left_motor_back
        // Motor port 1 = left_motor_front
        // Motor port 2 = right_motor_back
        // Motor port 3 = right_motor_front
        leftMotorBack   = hardwareMap.get(DcMotor.class, "left_motor_back");
        leftMotorFront  = hardwareMap.get(DcMotor.class, "left_motor_front");
        rightMotorBack  = hardwareMap.get(DcMotor.class, "right_motor_back");
        rightMotorFront = hardwareMap.get(DcMotor.class, "right_motor_front");

        // Brake on zero power
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse right side so all motors drive forward correctly.
        // If strafing goes the wrong direction, swap which side is reversed.
        leftMotorBack.setDirection(DcMotor.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotor.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotor.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);

        // Run without encoders for power-based driving
        leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized — ready to start");
        telemetry.update();

        waitForStart();

        // ── Main loop ──────────────────────────────────────────────────────────
        while (opModeIsActive()) {

            // ── Read Gamepad Inputs ────────────────────────────────────────────
            // Left  stick Y  → forward / backward  (negate: up = negative on gamepad)
            // Left  stick X  → strafe left / right
            // Right stick X  → rotate left / right
            double drive  =  gamepad1.left_stick_y;   // forward/back
            double strafe =  gamepad1.left_stick_x;   // left/right
            double rotate =  -gamepad1.right_stick_x;  // turn

            // ── Precision / Slow Mode ──────────────────────────────────────────
            // Hold right bumper for half-speed fine control
            double speedMult = gamepad1.right_bumper ? SPEED_MULTIPLIER * 0.5 : SPEED_MULTIPLIER;

            // ── Mecanum Wheel Power Equations ─────────────────────────────────
            //
            //   Mecanum rollers are at 45°, so each wheel gets a unique mix of
            //   drive, strafe, and rotate to produce omni-directional movement:
            //
            //   Front-Left  = drive + strafe + rotate
            //   Front-Right = drive - strafe - rotate
            //   Back-Left   = drive - strafe + rotate
            //   Back-Right  = drive + strafe - rotate
            //
            double frontLeftPower  = (drive + strafe + rotate) * speedMult;
            double frontRightPower = (drive - strafe - rotate) * speedMult;
            double backLeftPower   = (drive - strafe + rotate) * speedMult;
            double backRightPower  = (drive + strafe - rotate) * speedMult;

            // ── Normalize powers ───────────────────────────────────────────────
            // If any value exceeds 1.0, scale ALL motors down proportionally
            // so the intended drive direction is preserved at full stick deflection.
            double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            frontLeftPower  /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower   /= maxPower;
            backRightPower  /= maxPower;

            // Apply power to motors
            leftMotorFront.setPower(frontLeftPower);
            rightMotorFront.setPower(frontRightPower);
            leftMotorBack.setPower(backLeftPower);
            rightMotorBack.setPower(backRightPower);

            // ── Telemetry ──────────────────────────────────────────────────────
            telemetry.addData("Controls", "LS=Drive/Strafe | RS=Rotate | RB=Slow");
            telemetry.addLine();
            telemetry.addData("Drive",  "%.2f", drive);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Rotate", "%.2f", rotate);
            telemetry.addLine();
            telemetry.addData("FL Power", "%.2f", frontLeftPower);
            telemetry.addData("FR Power", "%.2f", frontRightPower);
            telemetry.addData("BL Power", "%.2f", backLeftPower);
            telemetry.addData("BR Power", "%.2f", backRightPower);
            telemetry.addData("Slow Mode", gamepad1.right_bumper ? "ON" : "OFF");
            telemetry.update();
        }

        // Stop all motors when OpMode ends
        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);
    }
}
package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * Intake2WheelsTeleop
 * -------------------------------------------------------------------------------------
 * WHAT THIS OP MODE DOES:
 *   Driver-controlled mecanum drive plus an intake motor. Controls:
 *     Left stick  — forward / backward / strafe
 *     Right stick — rotate
 *     Y button    — toggle intake direction (normal vs. reverse)
 *     Right trigger — run intake forward at trigger pressure
 *     Left trigger  — run intake backward at trigger pressure
 *
 * HARDWARE REQUIRED (names must match the Driver Hub robot config):
 *   Control Hub:
 *     - "front_left_motor", "front_right_motor", "back_left_motor", "back_right_motor"
 *   Expansion Hub:
 *     - "intake" (motor port 0) — the two-wheel intake
 * =====================================================================================
 */

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake 2 Wheels Teleop", group = "TeleOp")
public class Intake2WheelsTeleop extends LinearOpMode {

    // ── DRIVE & INTAKE ────────────────────────────
    private DcMotor front_left_motor;
    private DcMotor front_right_motor;
    private DcMotor back_left_motor;
    private DcMotor back_right_motor;
    private DcMotor intakeMotor;

    // ── STATES ────────────────────────────────────
    private boolean intakeReversed = false;
    private boolean yPrev          = false;

    @Override
    public void runOpMode() {
        TelemetryLog log = new TelemetryLog(PanelsTelemetry.INSTANCE.getTelemetry(), "Intake2WheelsTeleop");

        // ── DRIVE ─────────────────────────────────
        front_left_motor  = hardwareMap.get(DcMotor.class, "front_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        back_left_motor   = hardwareMap.get(DcMotor.class, "back_left_motor");
        back_right_motor  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Reverse left side so all wheels drive forward together
        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);

        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Expansion Hub: motor0=intake
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        log.debug("Status", "Ready");
        log.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {

            // ── MECANUM DRIVE ─────────────────────
            double drive  = -gamepad1.left_stick_y;   // forward/back
            double strafe =  gamepad1.left_stick_x;   // left/right strafing
            double turn   =  gamepad1.right_stick_x;  // rotation

            double flPower = drive + strafe + turn;
            double frPower = drive - strafe - turn;
            double blPower = drive - strafe + turn;
            double brPower = drive + strafe - turn;

            // Normalize so no value exceeds 1.0
            double max = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(blPower), Math.abs(brPower)));
            if (max > 1.0) {
                flPower /= max;
                frPower /= max;
                blPower /= max;
                brPower /= max;
            }

            front_left_motor.setPower(flPower);
            front_right_motor.setPower(frPower);
            back_left_motor.setPower(blPower);
            back_right_motor.setPower(brPower);

            // ── INTAKE TOGGLE (Y) ─────────────────
            boolean yCurr = gamepad1.y;
            if (yCurr && !yPrev) {
                intakeReversed = !intakeReversed;
                log.debug("Intake toggled to " + (intakeReversed ? "REVERSE" : "NORMAL"));
            }
            yPrev = yCurr;

            double dir = intakeReversed ? -1 : 1;

            if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(dir * gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(-dir * gamepad1.left_trigger);
            } else {
                intakeMotor.setPower(0);
            }

            // ── TELEMETRY ─────────────────────────
            log.debug("Intake", intakeReversed ? "REVERSE" : "NORMAL");
            log.addData("Intake Motor Power", intakeMotor.getPower());
            log.addData("FL Power", flPower);
            log.addData("FR Power", frPower);
            log.update(telemetry);
        }

        log.save();
    }
}
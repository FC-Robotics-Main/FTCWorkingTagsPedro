package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * DRIVE MOTOR TEST: runs ONE drive wheel at a time so you can see
 * which physical wheel each config name controls and whether it pushes.
 *
 * HOLD a button to run that motor FORWARD at TEST_POWER:
 *   X = FLM (front left)     Y = FRM (front right)
 *   A = BLM (back left)      B = BRM (back right)
 * Hold the LEFT BUMPER at the same time to run it in REVERSE.
 *
 * Directions match DuckTapeRobotics-1.0, so "forward" here = forward there.
 * Run it with the robot on the floor AND on blocks.
 */
@TeleOp(name = "Drive Motor Test", group = "Test")
public class DriveMotorTest extends LinearOpMode {

    private static final double TEST_POWER = 0.5;

    @Override
    public void runOpMode() {
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "FLM");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "FRM");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "BLM");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "BRM");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx[] motors = {fl, fr, bl, br};
        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Hold X=FL  Y=FR  A=BL  B=BR   (+ Left Bumper = reverse)");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double p = gamepad1.left_bumper ? -TEST_POWER : TEST_POWER;

            fl.setPower(gamepad1.x ? p : 0);
            fr.setPower(gamepad1.y ? p : 0);
            bl.setPower(gamepad1.a ? p : 0);
            br.setPower(gamepad1.b ? p : 0);

            telemetry.addLine("Hold X=FL  Y=FR  A=BL  B=BR   (+ Left Bumper = reverse)");
            telemetry.addData("FLM", "power %.2f  current %.2f A  port %d",
                    fl.getPower(), fl.getCurrent(CurrentUnit.AMPS), fl.getPortNumber());
            telemetry.addData("FRM", "power %.2f  current %.2f A  port %d",
                    fr.getPower(), fr.getCurrent(CurrentUnit.AMPS), fr.getPortNumber());
            telemetry.addData("BLM", "power %.2f  current %.2f A  port %d",
                    bl.getPower(), bl.getCurrent(CurrentUnit.AMPS), bl.getPortNumber());
            telemetry.addData("BRM", "power %.2f  current %.2f A  port %d",
                    br.getPower(), br.getCurrent(CurrentUnit.AMPS), br.getPortNumber());
            telemetry.update();
        }
    }
}
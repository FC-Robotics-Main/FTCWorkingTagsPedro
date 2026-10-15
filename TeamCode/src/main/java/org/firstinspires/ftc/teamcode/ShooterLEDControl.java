package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@TeleOp(name = "Shooter LED Control", group = "Robot")
public class ShooterLEDControl extends LinearOpMode {

    private I2cDeviceSynch prismI2c;
    private static final I2cAddr PRISM_ADDRESS = I2cAddr.create7bit(0x38); // Default Prism I2C address

    // Define example command bytes/artboard indices for your colors
    // (Check your Prism configuration software for exact register values or saved artboard slots)
    private static final int LED_CMD_REGISTER = 0; // Example register address
    private static final int COLOR_RED        = 0x01;
    private static final int COLOR_YELLOW     = 0x02;
    private static final int COLOR_GREEN      = 0x03;

    private double targetRpm = 0;

    @Override
    public void runOpMode() {
        // Initialize the Prism driver configured on I2C Bus 0
        prismI2c = hardwareMap.get(I2cDeviceSynch.class, "prism_i2c");
        prismI2c.setI2cAddress(PRISM_ADDRESS);
        prismI2c.engage();

        waitForStart();

        while (opModeIsActive()) {

            // Check shooter preset buttons
            if (gamepad1.x) {
                targetRpm = 2500;
                prismI2c.write8(LED_CMD_REGISTER, COLOR_RED); // Set LEDs to Red
            } else if (gamepad1.y) {
                targetRpm = 2400;
                prismI2c.write8(LED_CMD_REGISTER, COLOR_YELLOW); // Set LEDs to Yellow
            } else if (gamepad1.b) {
                targetRpm = 2300;
                prismI2c.write8(LED_CMD_REGISTER, COLOR_GREEN); // Set LEDs to Green
            }

            // Run your shooter motor logic using targetRpm here...

            telemetry.addData("Target RPM", targetRpm);
            telemetry.update();
        }
    }
}
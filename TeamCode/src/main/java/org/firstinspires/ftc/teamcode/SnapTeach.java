package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Snap Teach: put the robot exactly on the primary (back square to the cell, on the cell's center line,
 * where you hit 40 of 40), choose the alliance with X (red) or B (blue), and press A to save.
 * One teach covers all four cells. Re-teach only if the camera mount changes or gets bumped.
 * The TeleOp reads the saved file when it starts, so restart the TeleOp after teaching.
 * Report: 2026-10-27-Snap-To-Arc-Aim-Button-V05, sections 5.5 and 6.
 */
@TeleOp(name = "Snap Teach", group = "Test")
public class SnapTeach extends LinearOpMode {
    @Override
    public void runOpMode() {
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.setPollRateHz(100);
        ll.pipelineSwitch(0);
        ll.start();
        boolean blue = false;
        String saved = SnapAim.loadTeach() == null ? "never taught" : "taught (file exists)";

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) blue = false;
            if (gamepad1.b) blue = true;
            int firstId = blue ? 38 : 30;

            LLResult r = ll.getLatestResult();
            SnapAim.Reading rd = SnapAim.compute(r, firstId);
            telemetry.addData("Alliance (X red / B blue)", blue ? "BLUE" : "RED");
            telemetry.addData("Status", saved);
            if (r != null && r.isValid()) {
                for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
                    Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.MM);
                    telemetry.addData("Tag " + f.getFiducialId(),
                            "right %.0f  down %.0f  out %.0f mm", p.x, p.y, p.z);
                }
            }
            if (rd.ok) {
                telemetry.addData("Tags used", rd.ids);
                telemetry.addData("Distance (mm)", "%.1f", rd.distMm);
                telemetry.addData("Turn (deg, + = counter-clockwise)", "%.2f", rd.turnDeg);
                telemetry.addData("Around (deg)", "%.1f", rd.aroundDeg);
                telemetry.addData("Tag height (expect 1268 mm)", "%.0f", rd.tagHeightMm);
                telemetry.addData("Row length (expect 1.00)", "%.3f", rd.rowScale);
            } else {
                telemetry.addData("Not usable", rd.why);
            }
            telemetry.addLine("A = save the teach (robot must be exactly on the primary)");
            telemetry.update();

            if (gamepad1.a) {
                // Average 50 fresh readings (about 1-2 s), each using at least 3 tags.
                double sumD = 0, sumT = 0, lastTs = -1;
                int got = 0;
                ElapsedTime tm = new ElapsedTime();
                while (opModeIsActive() && got < 50 && tm.seconds() < 5) {
                    LLResult x = ll.getLatestResult();
                    if (x != null && x.getTimestamp() != lastTs) {
                        lastTs = x.getTimestamp();
                        SnapAim.Reading y = SnapAim.compute(x, firstId);
                        if (y.ok && y.tags >= 3) { sumD += y.distMm; sumT += y.turnDeg; got++; }
                    }
                    sleep(5);
                }
                if (got >= 30) {
                    SnapAim.saveTeach(sumD / got, sumT / got);
                    saved = String.format("SAVED: distance %.1f mm, turn %.2f deg (%d readings)",
                            sumD / got, sumT / got, got);
                } else {
                    saved = "NOT saved: only " + got + " good readings. Check the tags are in view.";
                }
                while (opModeIsActive() && gamepad1.a) sleep(10);   // wait for release
            }
        }
        ll.stop();
    }
}

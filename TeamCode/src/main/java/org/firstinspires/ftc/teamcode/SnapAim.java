package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * Snap-to-arc math (Option D). Turns one Limelight result into two numbers:
 *   distMm  = robot center to the aim point (the arc's center)
 *   turnDeg = how far to turn so the robot's BACK points at the aim point (+ = counter-clockwise from above)
 * The aim point is the spot on the floor plan directly under where the primary shot passes through the
 * cell's opening: AIM_DEPTH_MM behind the bottom lip, on the cell's center line. (It is NOT the middle of
 * the cell floor, which is 133 mm behind the lip.)
 * Robot directions: FRONT = intake side, BACK = shooter side (the camera is here), LEFT = the robot's own
 * left when facing its front, UP. All distances in mm.
 * Works the same for all four cells: it only uses each tag's slot (1-4) in the row.
 * Used by DuckTapeRobotics_2_1 (the arc snap) and SnapTeach. Report: 2026-10-27-Snap-To-Arc-Aim-Button-V05.
 */
public class SnapAim {

    // ---- Camera mount. Measure on the robot (report section 5.2). The teach press cancels small errors. ----
    // The camera is on the BACK and looks backward and up at the cell.
    public static final double CAM_BACK_MM   = 180.0; // robot center to the lens, toward the back
    public static final double CAM_LEFT_MM   = 0.0;   // + = lens toward the robot's left side
    public static final double CAM_UP_MM     = 255.0; // lens height above the tiles

    // Camera tilt up from level. 2026-10-29: changed from 44.0 to 40.3. With 44.0, Snap Teach showed a
    // tag height of 1330 mm instead of 1268 mm. Live camera readings from the primary spot give 1268 mm
    // at 40.3 degrees (about 18 mm of tag height per degree). After changing this, run Snap Teach again:
    // the saved teach was measured with the old tilt.
    public static final double CAM_PITCH_DEG = 40.3;

    // ---- Cell sticker, from the manual (TU06 Figures 9-15 to 9-17). Same on all four cells. ----
    // Slot 1-4, left to right in the camera picture. Slot index = (tag ID - 30) % 4.
    private static final double[] SLOT_OFFSET_MM = {-165.1, -69.85, 69.85, 165.1};
    private static final double LIP_FROM_ROW_MM = 158.2;  // bottom lip, from the tag row's middle toward you

    // ---- Aim. Picked by shooting (report section 6, step 2: try 50, 75 and 115). ----
    public static final double AIM_DEPTH_MM = 75.0;       // aim point this far behind the lip

    public static final String TEACH_FILE = "snap_teach.txt";   // lives in /sdcard/FIRST/settings/

    public static class Reading {
        public boolean ok;
        public String why = "";
        public int tags;
        public String ids = "";
        public double distMm;        // robot center to the aim point
        public double turnDeg;       // + = turn counter-clockwise so the back points at the aim point
        public double aroundDeg;     // degrees around the arc, + = robot is right of the cell's center line
        public double tagHeightMm;   // should be about 1268 (checks CAM_PITCH_DEG and CAM_UP_MM)
        public double rowScale;      // should be about 1.00 (checks the tag size setting)
    }

    /** firstId = 30 for RED (cells 30-33 and 34-37) or 38 for BLUE (38-41 and 42-45). */
    public static Reading compute(LLResult result, int firstId) {
        Reading out = new Reading();
        if (result == null || !result.isValid()) { out.why = "no camera result"; return out; }

        // Which of our two cells shows more tags? Cell number = (ID - 30) / 4.
        int[] count = new int[4];
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            int id = f.getFiducialId();
            if (id >= firstId && id < firstId + 8) count[(id - 30) / 4]++;
        }
        int cell = (firstId - 30) / 4;
        if (count[cell + 1] > count[cell]) cell++;

        // Camera to robot: each tag's position from the robot's center, in FRONT / LEFT / UP.
        // Limelight camera space: x = right in the picture, y = down, z = out of the lens.
        // The camera looks out the BACK, so "out of the lens" is toward the back and the picture's right
        // is the robot's left.
        double t = Math.toRadians(CAM_PITCH_DEG), st = Math.sin(t), ct = Math.cos(t);
        double[] s = new double[4], fr = new double[4], lf = new double[4], up = new double[4];
        int n = 0;
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            int id = f.getFiducialId();
            if (id < 30 || id > 45 || (id - 30) / 4 != cell || n == 4) continue;
            Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.MM);
            s[n]  = SLOT_OFFSET_MM[(id - 30) % 4];
            fr[n] = -CAM_BACK_MM - p.y * st - p.z * ct;   // camera to robot: + = toward the front
            lf[n] =  CAM_LEFT_MM + p.x;                   // camera to robot: + = robot's left
            up[n] =  CAM_UP_MM   - p.y * ct + p.z * st;   // camera to robot: + = up
            out.ids += id + " ";
            n++;
        }
        out.tags = n;
        if (n < 2) { out.why = (n == 0) ? "no tags (cell down?)" : "only 1 tag"; return out; }

        // Fit the row: each tag = row middle + (its slot offset) x (row direction).
        double sm = 0, fm = 0, lm = 0, um = 0;
        for (int i = 0; i < n; i++) { sm += s[i]; fm += fr[i]; lm += lf[i]; um += up[i]; }
        sm /= n; fm /= n; lm /= n; um /= n;
        double ss = 0, uf = 0, ul = 0;
        for (int i = 0; i < n; i++) {
            double ds = s[i] - sm;
            ss += ds * ds; uf += ds * (fr[i] - fm); ul += ds * (lf[i] - lm);
        }
        uf /= ss; ul /= ss;
        out.rowScale = Math.hypot(uf, ul);
        uf /= out.rowScale; ul /= out.rowScale;          // row direction, toward slot 4
        double cf = fm - sm * uf, cl = lm - sm * ul;      // row middle, seen from above
        out.tagHeightMm = um;

        // "Toward you": straight across the row, from the cell back toward the robot.
        double wf = ul, wl = -uf;
        if (wf * cf + wl * cl > 0) { wf = -wf; wl = -wl; }

        // The aim point: AIM_DEPTH_MM behind the lip, under where the primary shot goes through. It is behind the robot.
        double k = LIP_FROM_ROW_MM - AIM_DEPTH_MM;
        double af = cf + k * wf, al = cl + k * wl;
        out.distMm = Math.hypot(af, al);
        // Degrees around: + = robot is to the right of the cell's center line, as seen from the robot.
        out.aroundDeg = Math.toDegrees(Math.atan2(-(af * uf + al * ul), -(af * wf + al * wl)));
        // Turn so the BACK points at the aim point. The back faces -front, so measure from there.
        out.turnDeg = Math.toDegrees(Math.atan2(-al, -af));
        out.ok = true;
        return out;
    }

    public static void saveTeach(double distMm, double turnDeg) {
        File f = AppUtil.getInstance().getSettingsFile(TEACH_FILE);
        ReadWriteFile.writeFile(f, String.format(Locale.US, "%.2f,%.3f", distMm, turnDeg));
    }

    /** {taught distance mm, taught turn deg}, or null if Snap Teach has never been run on this Control Hub. */
    public static double[] loadTeach() {
        try {
            File f = AppUtil.getInstance().getSettingsFile(TEACH_FILE);
            String[] v = ReadWriteFile.readFile(f).trim().split(",");
            return new double[]{Double.parseDouble(v[0]), Double.parseDouble(v[1])};
        } catch (Exception e) {
            return null;
        }
    }
}

package org.firstinspires.ftc.teamcode;

/* =====================================================================================
 * TelemetryLog
 * -------------------------------------------------------------------------------------
 * WHAT THIS CLASS DOES:
 *   A wrapper around TelemetryManager that adds two things:
 *     1. In-match history panel — keeps a rolling list of the last 25 log entries
 *        and displays them on the Driver Station via PanelsTelemetry every update.
 *     2. CSV file logging — saves a complete timestamped run log to
 *        /sdcard/FIRST/logs/<opModeName>_<timestamp>.csv when save() is called.
 *
 * HOW TO USE:
 *   TelemetryLog log = new TelemetryLog(PanelsTelemetry.INSTANCE.getTelemetry(), "MyOp");
 *   log.addData("Key", value);   // same as telemetry.addData()
 *   log.debug("message");        // lightweight status message
 *   log.update(telemetry);       // call instead of telemetry.update()
 *   log.save();                  // call in stop() to write the CSV
 * =====================================================================================
 */

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Wraps TelemetryManager to add persistent history.
 * - Shows the last DISPLAY_LINES entries in FullPanels every update
 * - Saves a complete timestamped CSV to /sdcard/FIRST/logs/ on save()
 */
public class TelemetryLog {
    private static final int DISPLAY_LINES = 25;
    private static final String LOG_DIR = "/sdcard/FIRST/logs/";

    private final TelemetryManager telemetryM;
    private final ElapsedTime timer = new ElapsedTime();
    private final List<String> history = new ArrayList<>();
    private final String filepath;

    public TelemetryLog(TelemetryManager telemetryM, String opModeName) {
        this.telemetryM = telemetryM;
        String ts = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        this.filepath = LOG_DIR + opModeName + "_" + ts + ".csv";
    }

    public void addData(String key, double value) {
        record(key, String.format(Locale.US, "%.4f", value));
        telemetryM.addData(key, value);
    }

    public void addData(String key, Object value) {
        record(key, String.valueOf(value));
        telemetryM.addData(key, value);
    }

    public void debug(String message) {
        record("MSG", message);
        telemetryM.debug(message);
    }

    public void debug(String key, Object value) {
        record(key, String.valueOf(value));
        telemetryM.debug(key, value);
    }

    private void record(String key, String value) {
        history.add(String.format(Locale.US, "%.2f,%s,%s", timer.seconds(), key, value));
    }

    /**
     * Appends the recent history log to panels then flushes the current frame.
     * Call this instead of telemetryM.update(telemetry).
     */
    public void update(Telemetry telemetry) {
        int start = Math.max(0, history.size() - DISPLAY_LINES);
        telemetryM.debug("── Log (last " + Math.min(history.size(), DISPLAY_LINES) + " entries) ──");
        for (int i = start; i < history.size(); i++) {
            telemetryM.debug(history.get(i));
        }
        telemetryM.update(telemetry);
    }

    /**
     * Writes the full run log to /sdcard/FIRST/logs/<opModeName>_<timestamp>.csv.
     * Call this when the OpMode ends.
     */
    public void save() {
        try {
            new File(LOG_DIR).mkdirs();
            FileWriter writer = new FileWriter(filepath);
            writer.write("time_s,key,value\n");
            for (String entry : history) {
                writer.write(entry + "\n");
            }
            writer.close();
        } catch (IOException ignored) {
        }
    }
}

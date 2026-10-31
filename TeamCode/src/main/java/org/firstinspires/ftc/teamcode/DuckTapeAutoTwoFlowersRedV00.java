package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * RED Two Flowers V00 (2026-10-30).
 * All of the steps, poses and settings are in DuckTapeAutoTwoFlowersBaseV00.java. This file only says
 * "this is the RED alliance", so the shared code uses the RED pose column.
 */
@Autonomous(name = "RED Two Flowers V00", group = "Autonomous")
public class DuckTapeAutoTwoFlowersRedV00 extends DuckTapeAutoTwoFlowersBaseV00 {

    /*
     * WHAT: Says this op mode is NOT the blue alliance (so it is RED).
     * CALLED FROM: DuckTapeAutoTwoFlowersBaseV00.init().
     */
    @Override
    protected boolean isBlueAlliance() {
        return false;
    }
}

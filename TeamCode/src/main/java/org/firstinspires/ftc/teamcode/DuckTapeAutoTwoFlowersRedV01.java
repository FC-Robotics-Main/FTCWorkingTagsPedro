package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * RED Two Flowers V01 (2026-10-30).
 * All of the steps, poses and settings are in DuckTapeAutoTwoFlowersBaseV01.java. This file only says
 * "this is the RED alliance", so the shared code uses the RED pose column.
 */
@Autonomous(name = "RED Two Flowers V01", group = "Autonomous")
public class DuckTapeAutoTwoFlowersRedV01 extends DuckTapeAutoTwoFlowersBaseV01 {

    /*
     * WHAT: Says this op mode is NOT the blue alliance (so it is RED).
     * CALLED FROM: DuckTapeAutoTwoFlowersBaseV01.init().
     */
    @Override
    protected boolean isBlueAlliance() {
        return false;
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * BLUE Two Flowers V00 (2026-10-30).
 * All of the steps, poses and settings are in DuckTapeAutoTwoFlowersBaseV00.java. This file only says
 * "this is the BLUE alliance", so the shared code uses the BLUE pose column.
 */
@Autonomous(name = "BLUE Two Flowers V00", group = "Autonomous")
public class DuckTapeAutoTwoFlowersBlueV00 extends DuckTapeAutoTwoFlowersBaseV00 {

    /*
     * WHAT: Says this op mode IS the blue alliance.
     * CALLED FROM: DuckTapeAutoTwoFlowersBaseV00.init().
     */
    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}

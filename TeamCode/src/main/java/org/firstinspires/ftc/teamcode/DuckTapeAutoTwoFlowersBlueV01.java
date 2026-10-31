package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * BLUE Two Flowers V01 (2026-10-30).
 * All of the steps, poses and settings are in DuckTapeAutoTwoFlowersBaseV01.java. This file only says
 * "this is the BLUE alliance", so the shared code uses the BLUE pose column.
 */
@Autonomous(name = "BLUE Two Flowers V01", group = "Autonomous")
public class DuckTapeAutoTwoFlowersBlueV01 extends DuckTapeAutoTwoFlowersBaseV01 {

    /*
     * WHAT: Says this op mode IS the blue alliance.
     * CALLED FROM: DuckTapeAutoTwoFlowersBaseV01.init().
     */
    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}

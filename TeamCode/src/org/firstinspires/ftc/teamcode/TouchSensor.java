package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.GamePad;

/**
 * Uses the gamepad's a button to simulate a touch sensor.
 * */
public class TouchSensor {

    public boolean isPressed(GamePad g1)
    {
        return g1.a;
    }

}

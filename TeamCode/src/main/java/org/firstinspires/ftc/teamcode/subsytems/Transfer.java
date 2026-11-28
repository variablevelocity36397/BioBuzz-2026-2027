package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/** Transfer motor (reversed) that toggles between a set power and off. Call update() every loop. */
public class Transfer extends MotorToggle {
    public Transfer(HardwareMap hw, double power) {
        super(hw, "transfer", true, power);
    }
}

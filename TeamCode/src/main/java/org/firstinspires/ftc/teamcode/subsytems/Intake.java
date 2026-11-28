package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/** Both intake rollers (intake1 reversed), always on or off together. Call update() every loop. */
public class Intake {
    private final MotorToggle intake1, intake2;

    public Intake(HardwareMap hw) {
        intake1 = new MotorToggle(hw, "intake1", true, 1);
        intake2 = new MotorToggle(hw, "intake2", false, 1);
    }

    public void toggle() { set(!isOn()); }

    public void set(boolean on) {
        intake1.set(on);
        intake2.set(on);
    }

    public boolean isOn() { return intake1.isOn(); }

    public void update() {
        intake1.update();
        intake2.update();
    }
}

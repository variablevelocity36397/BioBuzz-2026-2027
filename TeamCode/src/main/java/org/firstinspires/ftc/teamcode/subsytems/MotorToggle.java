package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;

/** Motor that toggles between a set power (default full) and off. Call update() every loop. */
public class MotorToggle {
    private final DcMotor motor;
    private final double power;
    private boolean on;

    public MotorToggle(DcMotor motor) {
        this(motor, 1);
    }

    public MotorToggle(DcMotor motor, double power) {
        this.motor = motor;
        this.power = power;
    }

    public void toggle() { on = !on; }

    public void set(boolean on) { this.on = on; }

    public boolean isOn() { return on; }

    public void update() { motor.setPower(on ? power : 0); }
}

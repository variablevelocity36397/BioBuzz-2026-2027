package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public MotorToggle(HardwareMap hw, String name, boolean reversed, double power) {
        this(hw.get(DcMotor.class, name), power);
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void toggle() { on = !on; }

    public void set(boolean on) { this.on = on; }

    public boolean isOn() { return on; }

    public void update() { motor.setPower(on ? power : 0); }
}

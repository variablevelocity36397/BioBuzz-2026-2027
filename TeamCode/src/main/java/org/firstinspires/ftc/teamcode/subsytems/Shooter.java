package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/** Flywheel that holds a target velocity when on and coasts when off. Call update() every loop. */
public class Shooter {
    private final DcMotorEx motor;
    private final double step;
    private boolean on;
    private double velocity;

    public Shooter(DcMotorEx motor, double startVelocity, double step) {
        this.motor = motor;
        this.velocity = startVelocity;
        this.step = step;
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(230, 0, 0, 17.4570));
    }

    public void toggle() { on = !on; }

    public void set(boolean on) { this.on = on; }

    public boolean isOn() { return on; }

    public void faster() { velocity += step; }

    public void slower() { setVelocity(velocity - step); }

    public void setVelocity(double velocity) { this.velocity = Math.max(0, velocity); }

    public double getVelocity() { return velocity; }

    public double getActualVelocity() { return motor.getVelocity(); }

    public void update() {
        if (on) {
            motor.setVelocity(velocity);
        } else {
            motor.setPower(0);
        }
    }
}

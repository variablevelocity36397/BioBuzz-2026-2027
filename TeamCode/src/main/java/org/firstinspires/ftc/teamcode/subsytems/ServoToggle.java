package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.Servo;

/** Servo that toggles between two positions. Call update() every loop. */
public class ServoToggle {
    private final Servo servo;
    private final double offPosition, onPosition;
    private boolean on;

    public ServoToggle(Servo servo, double offPosition, double onPosition) {
        this.servo = servo;
        this.offPosition = offPosition;
        this.onPosition = onPosition;
    }

    public void toggle() { on = !on; }

    public void set(boolean on) { this.on = on; }

    public boolean isOn() { return on; }

    public double getPosition() { return servo.getPosition(); }

    public void update() { servo.setPosition(on ? onPosition : offPosition); }
}

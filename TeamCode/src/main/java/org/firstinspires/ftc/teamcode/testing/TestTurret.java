package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsytems.Turret;

@TeleOp(name = "Test Turret", group = "Test")
public class TestTurret extends OpMode {
    private static final double[] STEPS_DEG = {1, 5, 18};

    private Turret turret;
    private boolean aimOn;
    private int stepIndex;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        turret.center();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) turret.setRed(false);
        if (gamepad1.bWasPressed()) turret.setRed(true);
        telemetry.addData("Alliance (X blue / B red)", turret.isRed() ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            aimOn = !aimOn;
            if (!aimOn) turret.center();
        }
        if (gamepad1.yWasPressed()) stepIndex = (stepIndex + 1) % STEPS_DEG.length;

        if (aimOn) turret.aim();

        // + = left (CCW). setAngle clamps to the soft limits.
        double step = STEPS_DEG[stepIndex];
        if (gamepad1.leftBumperWasPressed()) turret.setAngle(turret.getAngleDeg() + step);
        if (gamepad1.rightBumperWasPressed()) turret.setAngle(turret.getAngleDeg() - step);

        telemetry.addData("Alliance", turret.isRed() ? "RED" : "BLUE");
        telemetry.addData("A - auto aim", aimOn);
        telemetry.addData("Y - step (deg)", step);
        telemetry.addData("LB/RB - turn left/right", "");
        telemetry.addData("Has target", turret.hasTarget());
        telemetry.addData("Tags in target", turret.getTagCount());
        telemetry.addData("tx", turret.getTx());
        telemetry.addData("Turret angle (deg)", turret.getAngleDeg());
        telemetry.update();
    }
}

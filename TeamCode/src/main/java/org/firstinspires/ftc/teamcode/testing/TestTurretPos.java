package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsytems.TurretPos;

/** Auto aim only. The turret servos are powered only while auto aim is on and an alliance tag is in view. */
@TeleOp(name = "Test Turret Pos", group = "Test")
public class TestTurretPos extends OpMode {
    private TurretPos turret;
    private boolean aimOn;

    @Override
    public void init() {
        turret = new TurretPos(hardwareMap);
        turret.setPowered(false);
    }

    @Override
    public void init_loop() {
        if (gamepad1.squareWasPressed()) turret.setRed(false);
        if (gamepad1.circleWasPressed()) turret.setRed(true);
        telemetry.addData("Alliance (Square blue / Circle red)", turret.isRed() ? "RED" : "BLUE");
        telemetry.addLine("Point the turret straight forward by hand before init.");
        telemetry.addData("Turret angle (deg)", turret.getAngleDeg());
        telemetry.addData("Servos", "unpowered");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.crossWasPressed()) aimOn = !aimOn;
        // Live tuning. Resets when the OpMode stops: copy good values into TurretPos.KP / TurretPos.DEADBAND_DEG.
        if (gamepad1.dpadLeftWasPressed()) turret.setKp(turret.getKp() - 0.01);
        if (gamepad1.dpadRightWasPressed()) turret.setKp(turret.getKp() + 0.01);
        if (gamepad1.leftBumperWasPressed()) turret.setKp(turret.getKp() + 0.1);
        if (gamepad1.dpadDownWasPressed()) turret.setDeadbandDeg(turret.getDeadbandDeg() - 0.25);
        if (gamepad1.dpadUpWasPressed()) turret.setDeadbandDeg(turret.getDeadbandDeg() + 0.25);

        // aim() only moves the turret when it sees a tag, and setAngle clamps to the soft limits.
        if (aimOn) turret.aim();
        boolean powered = aimOn && turret.hasTarget();
        turret.setPowered(powered);

        telemetry.addData("Alliance", turret.isRed() ? "RED" : "BLUE");
        telemetry.addData("Cross - auto aim", aimOn);
        telemetry.addData("Dpad Left/Right (0.01), Left bumper (+0.1) - KP", "%.2f", turret.getKp());
        telemetry.addData("Dpad Down/Up - deadband (deg)", "%.2f", turret.getDeadbandDeg());
        telemetry.addData("Servos", powered ? "powered" : "unpowered");
        telemetry.addLine("--- Turret (0 = forward, + = left) ---");
        double angle = turret.getAngleDeg();
        telemetry.addData("Turret angle (deg)", "%.2f", angle);
        telemetry.addData("Room to soft limits (deg)", "left %.1f / right %.1f", TurretPos.MAX_DEG - angle, angle - TurretPos.MIN_DEG);
        telemetry.addData("Soft limits (deg)", "%.0f to %.0f", TurretPos.MIN_DEG, TurretPos.MAX_DEG);
        telemetry.addData("Servo position", "%.4f", TurretPos.servoPosition(angle));
        telemetry.addData("Servo position at forward / limits", "%.4f / %.4f to %.4f",
                TurretPos.servoPosition(0), TurretPos.servoPosition(TurretPos.MIN_DEG), TurretPos.servoPosition(TurretPos.MAX_DEG));
        telemetry.addData("Center offset (deg)", TurretPos.CENTER_OFFSET_DEG);
        telemetry.addLine("--- Target ---");
        telemetry.addData("Has target", turret.hasTarget());
        telemetry.addData("Tag IDs seen", turret.getTagIds());
        telemetry.addData("Tags in target", turret.getTagCount());
        telemetry.addData("tx - tag offset left/right (deg)", "%.2f", turret.getTx());
        telemetry.addData("ty - tag offset up/down (deg)", "%.2f", turret.getTy());
        telemetry.addData("Tag area (% of image)", "%.2f", turret.getTargetArea());
        telemetry.addData("Correction per frame (deg)", "%.2f", TurretPos.AIM_SIGN * turret.getKp() * turret.getTx());
        telemetry.addData("Camera frame age (ms)", turret.getFrameAgeMs());
        telemetry.update();
    }
}

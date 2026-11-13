package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsytems.TurretPos;

import java.util.Arrays;
import java.util.List;

/**
 * Left / right bumper step the turret (both servos together) left / right by 1, 10, 18 or 45 deg (dpad up picks),
 * and shows every servo's position. Only use after each servo's direction is confirmed, or the two will fight.
 */
@TeleOp(name = "Test Directions", group = "Test")
public class TestDirections extends OpMode {
    // Keep in sync with MecanumDrive (flwheel) and TestTeleOp (shooter, intake1, transfer)
    private static final List<String> REVERSED = Arrays.asList("flwheel", "shooter", "intake1", "transfer");
    private static final double POWER = 0.3;
    private static final double RUN_SEC = 0.5;
    private static final double[] STEPS_DEG = {1, 10, 18, 45};

    private final ElapsedTime timer = new ElapsedTime();
    private int selected;
    private int stepIndex;
    private double runUntil;

    private ServoImplEx left, right;
    private double turretDeg;
    // Other position servos, read only (never moved here)
    private Servo adjustableIntake;
    private String expected = "-";
    private double adjustablePos = 0.5;

    @Override
    public void init() {

        left = hardwareMap.get(ServoImplEx.class, "turretleft");
        right = hardwareMap.get(ServoImplEx.class, "turretright");
        TurretPos.applyDirections(left, right);
        left.setPwmDisable();
        right.setPwmDisable();
        adjustableIntake = hardwareMap.get(Servo.class, "adjustableintake");
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {

        double curTurretPos = left.getPosition();
        double curTurretPos2 = right.getPosition();


        // Turret servos
        if (gamepad1.dpadUpWasPressed()) stepIndex = (stepIndex + 1) % STEPS_DEG.length;
        double step = STEPS_DEG[stepIndex];
        if (gamepad1.leftBumperWasPressed()) move(turretDeg + step);
        if (gamepad1.rightBumperWasPressed()) move(turretDeg - step);
        if (gamepad1.dpadDownWasPressed()) move(0);

        if (gamepad1.dpadLeftWasPressed()) {
            adjustablePos = adjustablePos - 0.05;
        }
        if (gamepad1.dpadRightWasPressed()) {
            adjustablePos = adjustablePos + 0.05;
        }

        adjustableIntake.setPosition(adjustablePos);

        telemetry.addLine();
        telemetry.addData("Left bumper", "turret LEFT one step");
        telemetry.addData("Right bumper", "turret RIGHT one step");
        telemetry.addData("Dpad Up - step (deg)", step);
        telemetry.addData("Dpad Down", "turret -> center");
        telemetry.addData("Turret angle (deg)", turretDeg);
        telemetry.addData("Turret should", expected);
        telemetry.addLine();
        telemetry.addLine("Servo positions (last commanded, NaN = never set this run)");
        telemetry.addData("adjustable pos: ", adjustablePos);
        telemetry.addData("turretleft", "%.3f (%s)", left.getPosition(), pwm(left));
        telemetry.addData("turretright", "%.3f (%s)", right.getPosition(), pwm(right));
        telemetry.addData("Turret center / limits", "%.3f / %.3f to %.3f",
                TurretPos.servoPosition(0), TurretPos.servoPosition(TurretPos.MIN_DEG), TurretPos.servoPosition(TurretPos.MAX_DEG));
        telemetry.addData("adjustableintake", "%.3f", adjustableIntake.getPosition());
        telemetry.addData("Cur left ServoPos", curTurretPos);
        telemetry.addData("Cur Right ServoPos", curTurretPos2);
        telemetry.update();
    }


    private static String pwm(ServoImplEx servo) {
        return servo.isPwmEnabled() ? "powered" : "unpowered";
    }

    // Both servos to the same angle, clamped to the turret soft limits.
    private void move(double deg) {
        turretDeg = Math.max(TurretPos.MIN_DEG, Math.min(TurretPos.MAX_DEG, deg));
        left.setPwmEnable();
        right.setPwmEnable();
        left.setPosition(TurretPos.servoPosition(turretDeg));
        right.setPosition(TurretPos.servoPosition(turretDeg));
        expected = turretDeg == 0 ? "be at CENTER" : String.format("be %.0f deg %s", Math.abs(turretDeg), turretDeg > 0 ? "LEFT" : "RIGHT");
    }
}

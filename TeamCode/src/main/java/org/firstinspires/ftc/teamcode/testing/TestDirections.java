package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

/** Dpad left / right steps the adjustable intake servo by 0.05 and shows its position. */
@TeleOp(name = "Test Directions", group = "Test")
public class TestDirections extends OpMode {
    // Keep in sync with MecanumDrive (flwheel) and TestTeleOp (shooter, intake1, transfer)
    private static final List<String> REVERSED = Arrays.asList("flwheel", "shooter", "intake1", "transfer");
    private static final double POWER = 0.3;
    private static final double RUN_SEC = 0.5;

    private final ElapsedTime timer = new ElapsedTime();
    private int selected;
    private double runUntil;

    private Servo adjustableIntake;
    private double adjustablePos = 0.5;

    @Override
    public void init() {
        adjustableIntake = hardwareMap.get(Servo.class, "adjustableintake");
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadLeftWasPressed()) {
            adjustablePos = adjustablePos - 0.05;
        }
        if (gamepad1.dpadRightWasPressed()) {
            adjustablePos = adjustablePos + 0.05;
        }

        adjustableIntake.setPosition(adjustablePos);

        telemetry.addData("adjustable pos: ", adjustablePos);
        telemetry.addData("adjustableintake", "%.3f", adjustableIntake.getPosition());
        telemetry.update();
    }

}

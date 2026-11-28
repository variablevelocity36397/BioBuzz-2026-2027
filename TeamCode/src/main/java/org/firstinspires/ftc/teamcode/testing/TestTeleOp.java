package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsytems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.MotorToggle;
import org.firstinspires.ftc.teamcode.subsytems.ServoToggle;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.Transfer;

@TeleOp(name = "Test TeleOp", group = "Test")
public class TestTeleOp extends OpMode {
    // Placeholder servo positions, tune on the robot
    private static final double ADJUSTABLE_INTAKE_OFF = 0.45;
    private static final double ADJUSTABLE_INTAKE_ON = 0.55;

    private static final double SLOW_SPEED = 0.2;
    private static final double TURN_SCALE = 0.8;
    private static final double SHOOTER_START_VELOCITY = 2000;
    private static final double SHOOTER_VELOCITY_STEP = 10;

    private MecanumDrive drive;
    private Shooter shooter;
    private MotorToggle intake1;
    private MotorToggle intake2;
    private Transfer transfer;
    private ServoToggle adjustableIntake;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap, SHOOTER_START_VELOCITY, SHOOTER_VELOCITY_STEP);
        intake1 = new MotorToggle(hardwareMap, "intake1", true, 1);
        intake2 = new MotorToggle(hardwareMap, "intake2", false, 1);
        transfer = new Transfer(hardwareMap, 0.75);
        adjustableIntake = new ServoToggle(hardwareMap.get(Servo.class, "adjustableintake"), ADJUSTABLE_INTAKE_OFF, ADJUSTABLE_INTAKE_ON);
    }

    @Override
    public void loop() {
        // hold right trigger for slow mode
        boolean slowMode = gamepad1.right_trigger > 0.2;
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * TURN_SCALE,
                slowMode ? SLOW_SPEED : 1.0);

        if (gamepad1.aWasPressed()) shooter.toggle();
        if (gamepad1.bWasPressed()) intake1.toggle();
        if (gamepad1.xWasPressed()) transfer.toggle();
        if (gamepad1.yWasPressed()) intake2.toggle();
        if (gamepad1.rightBumperWasPressed()) adjustableIntake.toggle();
        if (gamepad1.dpadUpWasPressed()) shooter.faster();
        if (gamepad1.dpadDownWasPressed()) shooter.slower();

        shooter.update();
        intake1.update();
        intake2.update();
        transfer.update();
        adjustableIntake.update();

        telemetry.addData("Left Stick", "drive / strafe");
        telemetry.addData("Right Stick X", "turn (" + TURN_SCALE + ")");
        telemetry.addData("RT (hold) - slow mode", slowMode);
        telemetry.addData("A - shooter", shooter.isOn());
        telemetry.addData("B - intake1", intake1.isOn());
        telemetry.addData("X - transfer", transfer.isOn());
        telemetry.addData("Y - intake2", intake2.isOn());
        telemetry.addData("RB - adjustable intake", adjustableIntake.isOn());
        telemetry.addData("DpadUp/Down - shooter velocity", shooter.getVelocity());
        telemetry.update();
    }
}

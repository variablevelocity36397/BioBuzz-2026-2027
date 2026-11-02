package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TestSystems extends OpMode {
    private DcMotor flwheel;
    private DcMotor frwheel;
    private DcMotor blwheel;
    private DcMotor brwheel;
    private DcMotorEx shooter;
    private DcMotor intake;
    private DcMotor transfer;
    private Servo flowerIntake;
    private Servo adjustableIntake;
    private Servo leftBrake;
    private Servo rightBrake;
    private Servo gate;

    // toggle flags
    private boolean shooterOn;
    private boolean intakeOn;
    private boolean transferOn;

    @Override
    public void init() {
        flwheel = hardwareMap.get(DcMotor.class, "flwheel");
        frwheel = hardwareMap.get(DcMotor.class, "frwheel");
        blwheel = hardwareMap.get(DcMotor.class, "blwheel");
        brwheel = hardwareMap.get(DcMotor.class, "brwheel");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        flowerIntake = hardwareMap.get(Servo.class, "flowerintake");
        adjustableIntake = hardwareMap.get(Servo.class, "adjustableintake");
        leftBrake = hardwareMap.get(Servo.class, "leftbrake");
        rightBrake = hardwareMap.get(Servo.class, "rightbrake");
        gate = hardwareMap.get(Servo.class, "gate");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {
        shooterOn = false;
        intakeOn = false;
        transferOn = false;
    }

    @Override
    public void loop() {
        // mecanum drive
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 0.8;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        flwheel.setPower((y + x + rx) / denominator);
        blwheel.setPower((y - x + rx) / denominator);
        frwheel.setPower((y - x - rx) / denominator);
        brwheel.setPower((y + x - rx) / denominator);

        if (gamepad1.rightBumperWasPressed()) {
            shooterOn = !shooterOn;
        }
        if (gamepad1.leftBumperWasPressed()) {
            intakeOn = !intakeOn;
        }
        if (gamepad1.dpadUpWasPressed()) {
            transferOn = !transferOn;
        }

        if (shooterOn) {
            shooter.setVelocity(2000);
        } else {
            shooter.setPower(0);
        }
        if (intakeOn) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
        if (transferOn) {
            transfer.setPower(1);
        } else {
            transfer.setPower(0);
        }

        telemetry.addData("Left Stick", "drive / strafe");
        telemetry.addData("Right Stick X", "turn (0.8)");
        telemetry.addData("RB - shooter", shooterOn);
        telemetry.addData("LB - intake", intakeOn);
        telemetry.addData("DpadUp - transfer", transferOn);
        telemetry.update();
    }
}

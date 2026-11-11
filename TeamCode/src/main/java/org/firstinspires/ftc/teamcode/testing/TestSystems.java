package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Systems", group = "Test")
public class TestSystems extends OpMode {
    // Placeholder servo positions, tune on the robot
    private static final double FLOWER_INTAKE_OFF = 0.45;
    private static final double FLOWER_INTAKE_ON = 0.55;
    private static final double ADJUSTABLE_INTAKE_OFF = 0.45;
    private static final double ADJUSTABLE_INTAKE_ON = 0.55;
    private static final double LEFT_BRAKE_OFF = 0.45;
    private static final double LEFT_BRAKE_ON = 0.55;
    private static final double RIGHT_BRAKE_OFF = 0.45;
    private static final double RIGHT_BRAKE_ON = 0.55;
    private static final double GATE_CLOSED = 0.45;
    private static final double GATE_OPEN = 0.55;

    private static final double SLOW_SPEED = 0.2;
    private static final double SHOOTER_VELOCITY_STEP = 10;

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
    private boolean gateOpen;
    private boolean flowerIntakeOn;
    private boolean adjustableIntakeOn;
    private boolean leftBrakeOn;
    private boolean rightBrakeOn;

    private double shooterVelocity = 2000;

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
    public void loop() {
        // mecanum drive, hold right trigger for slow mode
        boolean slowMode = gamepad1.right_trigger > 0.5;
        double speed = slowMode ? SLOW_SPEED : 1.0;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 0.8;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        flwheel.setPower((y + x + rx) / denominator * speed);
        blwheel.setPower((y - x + rx) / denominator * speed);
        frwheel.setPower((y - x - rx) / denominator * speed);
        brwheel.setPower((y + x - rx) / denominator * speed);

        if (gamepad1.aWasPressed()) shooterOn = !shooterOn;
        if (gamepad1.bWasPressed()) intakeOn = !intakeOn;
        if (gamepad1.xWasPressed()) transferOn = !transferOn;
        if (gamepad1.yWasPressed()) gateOpen = !gateOpen;
        if (gamepad1.leftBumperWasPressed()) flowerIntakeOn = !flowerIntakeOn;
        if (gamepad1.rightBumperWasPressed()) adjustableIntakeOn = !adjustableIntakeOn;
        if (gamepad1.dpadLeftWasPressed()) leftBrakeOn = !leftBrakeOn;
        if (gamepad1.dpadRightWasPressed()) rightBrakeOn = !rightBrakeOn;

        if (gamepad1.dpadUpWasPressed()) shooterVelocity += SHOOTER_VELOCITY_STEP;
        if (gamepad1.dpadDownWasPressed()) shooterVelocity = Math.max(0, shooterVelocity - SHOOTER_VELOCITY_STEP);



        if (shooterOn) {
            shooter.setVelocity(shooterVelocity);
        } else {
            shooter.setPower(0);
        }
        intake.setPower(intakeOn ? 1 : 0);
        transfer.setPower(transferOn ? 0.75 : 0);

        gate.setPosition(gateOpen ? GATE_OPEN : GATE_CLOSED);
        flowerIntake.setPosition(flowerIntakeOn ? FLOWER_INTAKE_ON : FLOWER_INTAKE_OFF);
        adjustableIntake.setPosition(adjustableIntakeOn ? ADJUSTABLE_INTAKE_ON : ADJUSTABLE_INTAKE_OFF);
        leftBrake.setPosition(leftBrakeOn ? LEFT_BRAKE_ON : LEFT_BRAKE_OFF);
        rightBrake.setPosition(rightBrakeOn ? RIGHT_BRAKE_ON : RIGHT_BRAKE_OFF);

        telemetry.addData("Left Stick", "drive / strafe");
        telemetry.addData("Right Stick X", "turn (0.8)");
        telemetry.addData("RT (hold) - slow mode", slowMode);
        telemetry.addData("A - shooter", shooterOn);
        telemetry.addData("B - intake", intakeOn);
        telemetry.addData("X - transfer", transferOn);
        telemetry.addData("Y - gate open", gateOpen);
        telemetry.addData("LB - flower intake", flowerIntakeOn);
        telemetry.addData("RB - adjustable intake", adjustableIntakeOn);
        telemetry.addData("DpadLeft - left brake", leftBrakeOn);
        telemetry.addData("DpadRight - right brake", rightBrakeOn);
        telemetry.addData("DpadUp/Down - shooter velocity", shooterVelocity);
        telemetry.update();
    }
}

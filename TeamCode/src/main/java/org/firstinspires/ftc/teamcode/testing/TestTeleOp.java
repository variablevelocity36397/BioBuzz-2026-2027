package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsytems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.MotorToggle;
import org.firstinspires.ftc.teamcode.subsytems.ServoToggle;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretPos;

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
    private MotorToggle transfer;
    private ServoToggle adjustableIntake;
    //private TurretPos turret;
    private boolean aimOn;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter = new Shooter(shooterMotor, SHOOTER_START_VELOCITY, SHOOTER_VELOCITY_STEP);
        DcMotor intake1Motor = hardwareMap.get(DcMotor.class, "intake1");
        intake1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1 = new MotorToggle(intake1Motor);
        intake2 = new MotorToggle(hardwareMap.get(DcMotor.class, "intake2"));
        DcMotor transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = new MotorToggle(transferMotor, 0.75);
        adjustableIntake = new ServoToggle(hardwareMap.get(Servo.class, "adjustableintake"), ADJUSTABLE_INTAKE_OFF, ADJUSTABLE_INTAKE_ON);
        //turret = new TurretPos(hardwareMap);
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
        if (gamepad1.dpadLeftWasPressed()) aimOn = !aimOn;

        // Turret only moves while auto aim is toggled on
        //if (aimOn) turret.aim();

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
        telemetry.addData("DpadLeft - turret auto aim", aimOn);
        //telemetry.addData("Turret has target", turret.hasTarget());
        //telemetry.addData("Turret tx", turret.getTx());
       // telemetry.addData("Turret angle (deg)", turret.getAngleDeg());
        telemetry.update();
    }
}

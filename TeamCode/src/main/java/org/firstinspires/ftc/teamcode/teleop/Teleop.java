package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsytems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.MotorToggle;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.Turret;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends OpMode {
    // Placeholder starting servo position, tune on the robot
    private static final double ADJUSTABLE_INTAKE_START = 0.45;
    private static final double ADJUSTABLE_INTAKE_STEP = 0.05;

    private static final double SLOW_SPEED = 0.2;
    private static final double TURN_SCALE = 0.8;
    private static final double SHOOTER_VELOCITY = 1090;
    private static final double SPIN_UP_TOLERANCE = 12.5;

    private MecanumDrive drive;
    private Shooter shooter;
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private double spinUpTime = -1; // -1 = not at speed yet
    private MotorToggle intake1;
    private MotorToggle intake2;
    private MotorToggle transfer;
    private Servo adjustableIntake;
    private double adjustableIntakePos = ADJUSTABLE_INTAKE_START;
    //private Turret turret;
    private boolean aimOn;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter = new Shooter(shooterMotor, SHOOTER_VELOCITY, 0); // fixed velocity, no step
        DcMotor intake1Motor = hardwareMap.get(DcMotor.class, "intake1");
        intake1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1 = new MotorToggle(intake1Motor);
        intake2 = new MotorToggle(hardwareMap.get(DcMotor.class, "intake2"));
        DcMotor transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = new MotorToggle(transferMotor, 0.75);
        // Not commanded here so the servo stays still during init
        adjustableIntake = hardwareMap.get(Servo.class, "adjustableintake");
        //turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {
        // hold R2 for slow mode
        boolean slowMode = gamepad1.right_trigger > 0.2;
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * TURN_SCALE,
                slowMode ? SLOW_SPEED : 1.0);

        if (gamepad1.aWasPressed()) {
            shooter.toggle();
            spinUpTimer.reset();
            spinUpTime = -1;
        }
        // wasPressed() consumes the press, so read it once and drive both intakes together
        if (gamepad1.bWasPressed()) {
            intake1.toggle();
            intake2.set(intake1.isOn());
        }
        if (gamepad1.xWasPressed()) transfer.toggle();
        // Servo is only commanded on a press, so it doesn't jump on start
        boolean servoUp = gamepad1.rightBumperWasPressed();
        boolean servoDown = gamepad1.leftBumperWasPressed();
        if (servoUp || servoDown) {
            adjustableIntakePos = Range.clip(adjustableIntakePos
                    + (servoUp ? ADJUSTABLE_INTAKE_STEP : -ADJUSTABLE_INTAKE_STEP), 0, 1);
            adjustableIntake.setPosition(adjustableIntakePos);
        }
        if (gamepad1.dpadLeftWasPressed()) aimOn = !aimOn;

        // Turret only moves while auto aim is toggled on
        //if (aimOn) turret.aim();

        shooter.update();
        intake1.update();
        intake2.update();
        transfer.update();

        double shooterError = shooter.getVelocity() - shooter.getActualVelocity();
        if (shooter.isOn() && spinUpTime < 0 && Math.abs(shooterError) < SPIN_UP_TOLERANCE) {
            spinUpTime = spinUpTimer.seconds();
        }

        telemetry.addData("Left Stick", "drive / strafe");
        telemetry.addData("Right Stick X", "turn (" + TURN_SCALE + ")");
        telemetry.addData("R2 (hold) - slow mode", slowMode);
        telemetry.addData("Cross - shooter", shooter.isOn());
        telemetry.addData("Circle - intake1", intake1.isOn());
        telemetry.addData("Square - transfer", transfer.isOn());
        telemetry.addData("Circle - intake2", intake2.isOn());
        telemetry.addData("R1/L1 - adjustable intake +/-", adjustableIntakePos);
        telemetry.addData("Shooter target velocity", shooter.getVelocity());
        telemetry.addData("Shooter actual velocity", "%.1f", shooter.getActualVelocity());
        telemetry.addData("Shooter error", "%.1f", shooterError);
        telemetry.addData("Shooter spin-up time", !shooter.isOn() ? "off"
                : spinUpTime < 0 ? String.format("spinning up... %.2f s", spinUpTimer.seconds())
                : String.format("%.2f s", spinUpTime));
        telemetry.addData("D-pad Left - turret auto aim", aimOn);
        //telemetry.addData("Turret has target", turret.hasTarget());
        //telemetry.addData("Turret tx", turret.getTx());
        // telemetry.addData("Turret angle (deg)", turret.getAngleDeg());
        telemetry.update();
    }
}

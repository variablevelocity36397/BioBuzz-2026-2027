package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsytems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.MotorToggle;
import org.firstinspires.ftc.teamcode.subsytems.ServoToggle;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretTrack;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends OpMode {
    // Start position, and the placeholder second position to tune on the robot
    private static final double ADJUSTABLE_INTAKE_START = 0.85;
    private static final double ADJUSTABLE_INTAKE_ALT = 0.45;

    private static final double SLOW_SPEED = 0.5;
    private static final double TURN_SCALE = 0.8;
    private static final double SHOOTER_VELOCITY = 1100;
    private static final double SPIN_UP_TOLERANCE = 12.5;

    private MecanumDrive drive;
    private Shooter shooter;
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private double spinUpTime = -1; // -1 = not at speed yet
    private MotorToggle intake1;
    private MotorToggle intake2;
    private MotorToggle transfer;
    private ServoToggle adjustableIntake;
    private PinpointLocalizer localizer;
    private TurretTrack turret;
    private final double[] kdSteps = {0.001, 0.0001};
    private int kdStepIndex;
    ElapsedTime transferTimer = new ElapsedTime();

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
        transfer = new MotorToggle(transferMotor, 0.7);
        // Moves to the start position during init
        adjustableIntake = new ServoToggle(hardwareMap.get(Servo.class, "adjustableintake"),
                ADJUSTABLE_INTAKE_START, ADJUSTABLE_INTAKE_ALT);
        adjustableIntake.update();
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        turret = new TurretTrack(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) turret.setAlliance(TurretTrack.Alliance.BLUE);
        if (gamepad1.bWasPressed()) turret.setAlliance(TurretTrack.Alliance.RED);
        telemetry.addData("ALLIANCE", turret.getAlliance());
        telemetry.addLine("X (Square) = BLUE, B (Circle) = RED");
        telemetry.addLine("Point the turret straight forward by hand before pressing play.");
        telemetry.addData("Start pose", turret.getAlliance().start);
        telemetry.update();
    }

    @Override
    public void start() {
        localizer.setPose(turret.getAlliance().start);
        turret.zero();
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
        if (gamepad1.xWasPressed()){
            transferTimer.reset();
            transfer.set(true);
        }
        // Checked every loop so the transfer stops 1 s after the press
        if (transfer.isOn() && transferTimer.seconds() >= 1) transfer.set(false);
        if (gamepad1.rightBumperWasPressed()) adjustableIntake.toggle();

        // Live KD tuning on gamepad2, resets when the OpMode stops: copy a good value into TurretTrack.KD
        if (gamepad2.bWasPressed()) kdStepIndex = (kdStepIndex + 1) % kdSteps.length;
        double kdStep = kdSteps[kdStepIndex];
        if (gamepad2.dpadUpWasPressed()) turret.setKd(turret.getKd() + kdStep);
        if (gamepad2.dpadDownWasPressed()) turret.setKd(turret.getKd() - kdStep);

        // Turret always tracks the goal (soft limits applied inside TurretTrack)
        localizer.update();
        Pose pose = localizer.state().pose();
        turret.update(pose, true);
        shooter.update();
        intake1.update();
        intake2.update();
        transfer.update();
        adjustableIntake.update();

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
        telemetry.addData("R1 - adjustable intake", adjustableIntake.getPosition());
        telemetry.addData("Shooter target velocity", shooter.getVelocity());
        telemetry.addData("Shooter actual velocity", "%.1f", shooter.getActualVelocity());
        telemetry.addData("Shooter error", "%.1f", shooterError);
        telemetry.addData("Shooter spin-up time", !shooter.isOn() ? "off"
                : spinUpTime < 0 ? String.format("spinning up... %.2f s", spinUpTimer.seconds())
                : String.format("%.2f s", spinUpTime));
        if (turret.isAtLimit()) telemetry.addLine("!!! TURRET AT SOFT LIMIT !!!");
        telemetry.addData("Alliance", turret.getAlliance());
        telemetry.addData("GP2 Dpad Up/Down - KD", "%.4f", turret.getKd());
        telemetry.addData("GP2 B (Circle) - KD step", "%.4f", kdStep);
        telemetry.addData("Turret angle / target (deg)", "%.1f / %.1f", turret.getAngleDeg(), turret.getTarget());
        telemetry.update();
    }
}

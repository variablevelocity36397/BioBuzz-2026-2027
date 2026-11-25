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
import org.firstinspires.ftc.teamcode.subsytems.TurretLimelight;
import org.firstinspires.ftc.teamcode.subsytems.TurretTrack;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends OpMode {
    // Start position, and the placeholder second position to tune on the robot
    private static final double ADJUSTABLE_INTAKE_START = 0.90;
    private static final double ADJUSTABLE_INTAKE_ALT = 0.50;

    private static final double SLOW_SPEED = 0.45;
    private static final double TURN_SCALE = 0.85;
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
    private TurretLimelight turret;
    private boolean turretTracking = true;
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
        turret = new TurretLimelight(hardwareMap); // servos unpowered until start()
        turret.resetEncoder(); // assumes the turret faces forward at init
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) turret.setAlliance(TurretTrack.Alliance.BLUE);
        if (gamepad1.bWasPressed()) turret.setAlliance(TurretTrack.Alliance.RED);
        telemetry.addData("ALLIANCE", turret.getAlliance());
        telemetry.addLine("X (Square) = BLUE, B (Circle) = RED");
        telemetry.addLine("Point the turret straight forward by hand before pressing play.");
        telemetry.addData("Start pose", turret.getAlliance().start);
        telemetry.addData("Turret angle from init (deg)", "%.1f", turret.getAngleDeg());
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
        // Transfer only runs while the shooter is on
        if (gamepad1.xWasPressed() && shooter.isOn()) {
            transferTimer.reset();
            transfer.set(true);
        }
        // Checked every loop so the transfer stops 1 s after the press, or right away if the shooter turns off
        if (transfer.isOn() && (transferTimer.seconds() >= 1 || !shooter.isOn())) transfer.set(false);
        if (gamepad1.rightBumperWasPressed()) adjustableIntake.toggle();

        // Limelight when tags are seen, odometry otherwise. Driver override: gamepad2 A turns tracking off (turret holds forward).
        // Soft limits apply in every mode.
        if (gamepad2.aWasPressed()) turretTracking = !turretTracking;
        localizer.update();
        Pose pose = localizer.state().pose();
        turret.update(pose, turretTracking);
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
        if (turret.isEncoderFault()) telemetry.addLine("!!! TURRET ENCODER FAULT: turret stopped, restart OpMode !!!");
        if (turret.isAtLimit()) telemetry.addLine("!!! TURRET AT SOFT LIMIT !!!");
        telemetry.addData("Alliance", turret.getAlliance());
        telemetry.addData("GP2 Cross - turret tracking", turretTracking ? "ON" : "OFF (hold forward)");
        telemetry.addData("Turret mode", turret.getMode());
        telemetry.addData("Robot pose x / y / heading (deg)", "%.1f / %.1f / %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Turret angle / target (deg)", "%.1f / %.1f", turret.getAngleDeg(), turret.getTarget());
        telemetry.update();
    }
}

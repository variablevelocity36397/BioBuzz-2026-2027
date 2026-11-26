package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.EnumSet;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsytems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.MotorToggle;
import org.firstinspires.ftc.teamcode.subsytems.ServoToggle;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretLimelight;
import org.firstinspires.ftc.teamcode.subsytems.TurretTrack;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends OpMode {
    // Starts down (init), R1 toggles up. Set on robot 2026-11-25
    private static final double ADJUSTABLE_INTAKE_DOWN = 0.635;
    private static final double ADJUSTABLE_INTAKE_UP = 0.275;

    private static final double SLOW_SPEED = 0.45;
    private static final double TURN_SCALE = 0.85;
    private static final double SHOOTER_VELOCITY = 1100;
    private static final double SPIN_UP_TOLERANCE = 12.5;
    private static final double MANUAL_TURRET_POWER = 0.35; // tune on robot (breaks away at ~0.07)
    private static final double TRIGGER_HOLD_S = 0.25;
    // Garden poses, same as SimpleBlueAuto / SimpleRedAuto (red = blue rotated 180 deg about the field center)
    private static final Pose BLUE_GARDEN = new Pose(130, 133.1, 0);
    private static final Pose RED_GARDEN = new Pose(144 - 130, 144 - 133.1, Math.PI);

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
    private boolean turretManual;
    private final ElapsedTime trackHold = new ElapsedTime(), zeroHold = new ElapsedTime();
    private boolean trackHoldDone, zeroHoldDone; // each hold fires once
    ElapsedTime transferTimer = new ElapsedTime();
    // Shot snapshot: taken on the X press, then the whole 1 s transfer is tracked. Held on screen until the next shot.
    private boolean shotStarted;
    private String shotSnapshot = "none yet";
    private final EnumSet<TurretLimelight.Mode> shotModes = EnumSet.noneOf(TurretLimelight.Mode.class);
    private double shotMaxError;
    private int shotMinTags;

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
                ADJUSTABLE_INTAKE_DOWN, ADJUSTABLE_INTAKE_UP);
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

    @SuppressLint("DefaultLocale")
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
            shotStarted = true; // snapshot after this loop's turret update
            transfer.set(true);
        }
        // Checked every loop so the transfer stops 1 s after the press, or right away if the shooter turns off
        if (transfer.isOn() && (transferTimer.seconds() >= 1 || !shooter.isOn())) transfer.set(false);
        if (gamepad1.rightBumperWasPressed()) adjustableIntake.toggle();

        // Limelight when tags are seen, odometry otherwise. Driver override: gamepad2 A turns tracking off (turret holds forward).
        // Soft limits apply in every mode.
        if (gamepad2.aWasPressed()) turretTracking = !turretTracking;
        // GP2: a bumper press takes manual control, hold L2 0.25 s = back to auto track, hold R2 0.25 s = re-zero turret
        if (gamepad2.leftBumperWasPressed() | gamepad2.rightBumperWasPressed()) turretManual = true;
        if (gamepad2.left_trigger < 0.5) { trackHold.reset(); trackHoldDone = false; }
        else if (!trackHoldDone && trackHold.seconds() >= TRIGGER_HOLD_S) {
            trackHoldDone = true;
            turretManual = false;
            turretTracking = true;
        }
        if (gamepad2.right_trigger < 0.5) { zeroHold.reset(); zeroHoldDone = false; }
        else if (!zeroHoldDone && zeroHold.seconds() >= TRIGGER_HOLD_S) {
            zeroHoldDone = true;
            turret.zero(); // current position = forward; soft limits now count from here
        }
        // Odometry reset with the robot parked in the garden
        if (gamepad2.dpadDownWasPressed()) localizer.setPose(RED_GARDEN);
        if (gamepad2.dpadUpWasPressed()) localizer.setPose(BLUE_GARDEN);
        localizer.update();
        Pose pose = localizer.state().pose();
        if (turretManual) {
            turret.manual((gamepad2.left_bumper ? MANUAL_TURRET_POWER : 0) - (gamepad2.right_bumper ? MANUAL_TURRET_POWER : 0));
        } else {
            turret.update(pose, turretTracking);
        }
        if (shotStarted) {
            shotStarted = false;
            shotSnapshot = String.format("%s | tags %s(%d) | tx %.1f | angle %.1f / target %.1f | pose %.1f, %.1f, %.1f",
                    turret.getMode(), turret.getCamera().getTagIds(), turret.getCamera().getTagCount(),
                    turret.getCamera().getTx(), turret.getAngleDeg(), turret.getTarget(),
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            shotModes.clear();
            shotMaxError = 0;
            shotMinTags = Integer.MAX_VALUE;
        }
        if (transfer.isOn()) { // balls can leave any time in the 1 s transfer
            shotModes.add(turret.getMode());
            shotMaxError = Math.max(shotMaxError, Math.abs(turret.getError()));
            shotMinTags = Math.min(shotMinTags, turret.getCamera().getTagCount());
        }
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
        telemetry.addData("GP2 L1/R1 - manual turret (left/right)", turretManual ? "MANUAL" : "auto");
        telemetry.addLine("GP2 hold L2 - back to auto track | hold R2 - zero turret (point forward first)");
        telemetry.addLine("GP2 Dpad Down - odo to RED garden | Dpad Up - odo to BLUE garden");
        telemetry.addData("Turret mode", turret.getMode());
        // Red: 34-37 = audience-side cell, 30-33 = far cell. Blue: 38-41 = audience side, 42-45 = far.
        telemetry.addData("Camera tags used (count)", "%s(%d)", turret.getCamera().getTagIds(), turret.getCamera().getTagCount());
        telemetry.addData("Last shot (at X)", shotSnapshot);
        if (!shotModes.isEmpty()) telemetry.addData("Last shot during transfer", "modes %s | max error %.1f deg | min tags %d",
                shotModes, shotMaxError, shotMinTags);
        telemetry.addData("Robot pose x / y / heading (deg)", "%.1f / %.1f / %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Aim correction (deg)", "%.1f", turret.getAimCorrection(pose));
        telemetry.addData("Turret angle / target (deg)", "%.1f / %.1f", turret.getAngleDeg(), turret.getTarget());
        telemetry.update();
    }
}

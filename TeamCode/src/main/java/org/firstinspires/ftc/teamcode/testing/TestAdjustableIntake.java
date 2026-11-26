package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsytems.MotorToggle;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretTrack;

/**
 * GP1: dpad up/down steps the adjustable intake by 0.025, left/right by 0.05. Init: X = blue, B = red.
 * GP2: TeleOp shooter / intakes / transfer controls.
 * Turret servos are never powered (move it by hand); only its encoder is read, zeroed at start.
 * Telemetry shows the Pinpoint pose and the angle the code would aim at, for measuring new aim points.
 */
@TeleOp(name = "Test Adjustable Intake", group = "Test")
public class  TestAdjustableIntake extends OpMode {
    private static final double SHOOTER_VELOCITY = 1100; // same as Teleop

    private Servo adjustableIntake;
    private double pos = 0.630; // Teleop down (0.635) minus 0.005
    private Shooter shooter;
    private MotorToggle intake1, intake2, transfer;
    private final ElapsedTime transferTimer = new ElapsedTime();
    private TurretTrack turret;
    private PinpointLocalizer localizer;

    @Override
    public void init() {
        adjustableIntake = hardwareMap.get(Servo.class, "adjustableintake");
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter = new Shooter(shooterMotor, SHOOTER_VELOCITY, 0);
        DcMotor intake1Motor = hardwareMap.get(DcMotor.class, "intake1");
        intake1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1 = new MotorToggle(intake1Motor);
        intake2 = new MotorToggle(hardwareMap.get(DcMotor.class, "intake2"));
        DcMotor transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = new MotorToggle(transferMotor, 0.7);
        // Constructor leaves the servos PWM-disabled (slack). zero()/update()/aimAt() would power them, so only the
        // encoder reads and aimAngle() (math only) are used here.
        turret = new TurretTrack(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) turret.setAlliance(TurretTrack.Alliance.BLUE);
        if (gamepad1.bWasPressed()) turret.setAlliance(TurretTrack.Alliance.RED);
        telemetry.addData("ALLIANCE", turret.getAlliance());
        telemetry.addLine("X (Square) = BLUE, B (Circle) = RED");
        telemetry.addData("Start pose", turret.getAlliance().start);
        telemetry.update();
    }

    @Override
    public void start() {
        turret.resetEncoder();
        localizer.setPose(turret.getAlliance().start);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) pos += 0.025;
        if (gamepad1.dpadDownWasPressed()) pos -= 0.025;
        if (gamepad1.dpadRightWasPressed()) pos += 0.05;
        if (gamepad1.dpadLeftWasPressed()) pos -= 0.05;
        pos = Range.clip(pos, 0, 1); // keep the stored value from drifting past the servo range
        adjustableIntake.setPosition(pos);

        // Same as Teleop, on gamepad2
        if (gamepad2.aWasPressed()) shooter.toggle();
        if (gamepad2.bWasPressed()) {
            intake1.toggle();
            intake2.set(intake1.isOn());
        }
        if (gamepad2.xWasPressed() && shooter.isOn()) {
            transferTimer.reset();
            transfer.set(true);
        }
        if (transfer.isOn() && (transferTimer.seconds() >= 1 || !shooter.isOn())) transfer.set(false);

        shooter.update();
        intake1.update();
        intake2.update();
        transfer.update();

        localizer.update();
        Pose pose = localizer.state().pose();
        double angle = turret.getAngleDeg(), aim = turret.aimAngle(pose);

        telemetry.addData("GP1 dpad - adjustableintake", "%.3f", pos);
        telemetry.addData("GP2 Cross - shooter", shooter.isOn());
        telemetry.addData("GP2 Circle - intakes", intake1.isOn());
        telemetry.addData("GP2 Square - transfer (1 s, shooter on)", transfer.isOn());
        telemetry.addData("Shooter actual velocity", "%.1f", shooter.getActualVelocity());
        telemetry.addData("Alliance", turret.getAlliance());
        telemetry.addData("Pose x / y / heading (deg)", "%.1f / %.1f / %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Turret angle (deg)", "%.1f", angle);
        telemetry.addData("Code would aim (deg)", "%.1f", aim);
        telemetry.addData("Code minus turret (deg)", "%.1f", TurretTrack.normalize(aim - angle));
        telemetry.addData("Aim correction used (deg)", "%.1f", turret.aimCorrection(pose));
        telemetry.update();
    }
}

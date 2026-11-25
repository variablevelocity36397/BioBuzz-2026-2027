package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsytems.MotorToggle;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class SimpleBlueAuto extends OpMode {

    private enum State { SPIN_UP, FEED, GARDEN, PARK, DONE }

    private static final double SHOOTER_VELOCITY = 1100;
    private static final double SPIN_UP_TOLERANCE = 20;
    private static final double TRANSFER_POWER = 0.7;
    private static final double ADJUSTABLE_INTAKE_START = 0.85;
    private static final double SHOOT_TIME = 1;     // seconds of transfer before transfer + shooter turn off
    private static final double GARDEN_DELAY = 0.4; // seconds of transfer before garden path starts
    private static final double SPIN_UP_TIMEOUT = 3; // feed anyway if flywheel isn't at speed by then

    private Follower follower;
    private Shooter shooter;
    private MotorToggle transfer;
    private MotorToggle intake1;
    private MotorToggle intake2;
    private Servo adjustableIntake;
    private final ElapsedTime transferTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private State state;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(85.7848, 133.1263, 0);
    private final Pose garden = poseFactory.of(130, 133.1, 0);
    private final Pose parkStart = poseFactory.of(130, 133.1, 0);
    private final Pose park = poseFactory.of(127.5047, 45.1531, 145);
    private final Pose parkControl1 = poseFactory.of(105.2935, 92.8837, 0);

    // Entering a state runs its action once
    private void setState(State next) {
        state = next;
        switch (next) {
            case SPIN_UP: spinUpTimer.reset(); break;
            case FEED:    transfer.set(true); transferTimer.reset(); break;
            case GARDEN:  follower.follow(garden()); break;
            case PARK:    follower.follow(park());   break;
            case DONE:    shooter.set(false); transfer.set(false); intake1.set(false); intake2.set(false); break;
        }
    }

    private void updateStateMachine() {
        switch (state) {
            case SPIN_UP:
                if (Math.abs(SHOOTER_VELOCITY - shooter.getActualVelocity()) <= SPIN_UP_TOLERANCE
                        || spinUpTimer.seconds() >= SPIN_UP_TIMEOUT) setState(State.FEED);
                break;
            case FEED:   if (transferTimer.seconds() >= GARDEN_DELAY) setState(State.GARDEN); break;
            case GARDEN: if (!follower.isBusy()) setState(State.PARK); break;
            case PARK:   if (!follower.isBusy()) setState(State.DONE); break;
            case DONE:   break;
        }
    }

    // Runs alongside the garden path: transfer + shooter off SHOOT_TIME after transfer started
    private void updateShooting() {
        if (transfer.isOn() && transferTimer.seconds() >= SHOOT_TIME) {
            transfer.set(false);
            shooter.set(false);
        }
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter = new Shooter(shooterMotor, SHOOTER_VELOCITY, 0); // fixed velocity, no step
        DcMotor transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = new MotorToggle(transferMotor, TRANSFER_POWER);
        DcMotor intake1Motor = hardwareMap.get(DcMotor.class, "intake1");
        intake1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1 = new MotorToggle(intake1Motor);
        intake2 = new MotorToggle(hardwareMap.get(DcMotor.class, "intake2"));
        adjustableIntake = hardwareMap.get(Servo.class, "adjustableintake");
    }

    @Override
    public void start() {
        adjustableIntake.setPosition(ADJUSTABLE_INTAKE_START);
        shooter.set(true);
        intake1.set(true);
        intake2.set(true);
        setState(State.SPIN_UP);
    }

    @Override
    public void loop() {
        follower.update();
        updateStateMachine();
        updateShooting();
        shooter.update();
        transfer.update();
        intake1.update();
        intake2.update();

        telemetry.addData("state", state);
        telemetry.addData("shooter velocity", shooter.getActualVelocity());
        telemetry.addData("transfer", transfer.isOn());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }
    }

    public Path garden() {
        return Paths.line(start, garden).linear(start, garden);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}

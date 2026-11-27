package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsytems.MotorToggle;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretLimelight;
import org.firstinspires.ftc.teamcode.subsytems.TurretTrack;

/** Blue, nectar side solo (RedYesNectarSolo rotated 180 deg about field center): three shots (preload, garden, flower), turret tracking the whole auto. Paths, poses and headings from the Turtle Tracer export. */
@Autonomous(name = "BlueYesNectarSolo", group = "Autonomous")
public class BlueYesNectarSolo extends OpMode {

    private enum State {
        SPIN_UP_1, FEED_1, INTAKE_GARDEN, GARDEN_WAIT, DRIVE_TO_OTHER_SIDE,
        SPIN_UP_2, FEED_2, LINE_UP_FLOWER, INTAKE_FLOWER, FLOWER_WAIT, SPIN_UP_3, FEED_3, PARK, DONE
    }

    private static final double SHOOTER_VELOCITY = 1100;
    private static final double SPIN_UP_TOLERANCE = 20;
    private static final double SPIN_UP_TIMEOUT = 2.5; // feed anyway if the flywheel isn't at speed by then
    private static final double SETTLE_TIME = 1;       // extra wait once at speed (not after a timeout)
    private static final double TRANSFER_POWER = 0.7;
    private static final double FEED_TIME = 1;         // seconds of transfer per shot
    private static final double GARDEN_WAIT_TIME = 1;   // seconds of intake at the garden before driving on
    private static final double FLOWER_WAIT_TIME = 1.5; // seconds of intake at the flower before shooting

    private Follower follower;
    private Shooter shooter;
    private MotorToggle transfer;
    private MotorToggle intake1;
    private MotorToggle intake2;
    private TurretLimelight turret;
    private final ElapsedTime timer = new ElapsedTime(); // states run one at a time, so one timer covers them all
    private double settledAt; // timer seconds when the shooter first got within tolerance, -1 = not yet
    private State state;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.807, 134.000, 0.000);
    private final Pose gardenStart = poseFactory.of(82.807, 134.000, 180.0);
    private final Pose garden = poseFactory.of(133.000, 134.000, 180.0);
    private final Pose otherSideControl1 = poseFactory.of(110.926, 106.195, 180.0);
    private final Pose otherSideControl2 = poseFactory.of(130.811, 47.537, 180.0);
    private final Pose otherSide = poseFactory.of(110.360, 15.060, 180.0);
    private final Pose flowerLineUp = poseFactory.of(104.870, 24.167, 180.0);
    private final Pose flower = poseFactory.of(104.870, 19.452, 180.0);
    private final Pose park = poseFactory.of(137.503, 25.362, 180.0);

    // Entering a state runs its action once
    private void setState(State next) {
        state = next;
        switch (next) {
            case SPIN_UP_1: case SPIN_UP_2: case SPIN_UP_3:
                shooter.set(true);
                timer.reset();
                settledAt = -1;
                break;
            case FEED_1: case FEED_2: case FEED_3:
                transfer.set(true);
                timer.reset();
                break;
            case INTAKE_GARDEN:       follower.follow(intakeGarden()); break;
            case GARDEN_WAIT: case FLOWER_WAIT: timer.reset(); break;
            case DRIVE_TO_OTHER_SIDE: follower.follow(driveToOtherSide()); break;
            case LINE_UP_FLOWER:      follower.follow(lineUpFlower()); break;
            case INTAKE_FLOWER:       follower.follow(intakeFlower()); break;
            case PARK:                follower.follow(parkPath()); break;
            case DONE:                shooter.set(false); transfer.set(false); break; // intakes stay on until the auto ends
        }
    }

    private void updateStateMachine() {
        switch (state) {
            case SPIN_UP_1: if (atSpeed()) setState(State.FEED_1); break;
            case SPIN_UP_2: if (atSpeed()) setState(State.FEED_2); break;
            case SPIN_UP_3: if (atSpeed()) setState(State.FEED_3); break;
            case FEED_1:    if (fed()) setState(State.INTAKE_GARDEN); break;
            case FEED_2:    if (fed()) setState(State.LINE_UP_FLOWER); break;
            case FEED_3:    if (fed()) setState(State.PARK);          break;
            case INTAKE_GARDEN:       if (!follower.isBusy()) setState(State.GARDEN_WAIT);         break;
            case GARDEN_WAIT:         if (timer.seconds() >= GARDEN_WAIT_TIME) setState(State.DRIVE_TO_OTHER_SIDE); break;
            case DRIVE_TO_OTHER_SIDE: if (!follower.isBusy()) setState(State.SPIN_UP_2);           break;
            case LINE_UP_FLOWER:      if (!follower.isBusy()) setState(State.INTAKE_FLOWER);       break;
            case INTAKE_FLOWER:       if (!follower.isBusy()) setState(State.FLOWER_WAIT);         break;
            case FLOWER_WAIT:         if (timer.seconds() >= FLOWER_WAIT_TIME) setState(State.SPIN_UP_3); break;
            case PARK:                if (!follower.isBusy()) setState(State.DONE);                break;
            case DONE:                break;
        }
    }

    // At speed, then SETTLE_TIME more; or the timeout, with no extra wait
    private boolean atSpeed() {
        double t = timer.seconds();
        if (settledAt < 0 && Math.abs(SHOOTER_VELOCITY - shooter.getActualVelocity()) <= SPIN_UP_TOLERANCE) settledAt = t;
        return (settledAt >= 0 && t - settledAt >= SETTLE_TIME) || t >= SPIN_UP_TIMEOUT;
    }

    // True once the shot is done; transfer and shooter turn off
    private boolean fed() {
        if (timer.seconds() < FEED_TIME) return false;
        transfer.set(false);
        shooter.set(false);
        return true;
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

        // Built after the follower: the turret reads the brwheel drive motor's encoder
        turret = new TurretLimelight(hardwareMap); // servos unpowered until start()
        turret.setAlliance(TurretTrack.Alliance.BLUE);
        turret.resetEncoder(); // assumes the turret faces forward at init
    }

    @Override
    public void init_loop() {
        follower.localizer.update(); // reads the Pinpoint only, never drives the wheels during init
        showPose();
        telemetry.addLine("Point the turret straight forward by hand before pressing play.");
        telemetry.addData("Turret angle from init (deg)", "%.1f", turret.getAngleDeg());
    }

    @Override
    public void start() {
        turret.zero();
        intake1.set(true); // intakes run the whole auto
        intake2.set(true);
        setState(State.SPIN_UP_1);
    }

    @Override
    public void loop() {
        follower.update();
        updateStateMachine();
        turret.update(follower.pose(), true); // tracks the whole auto; soft limits apply inside
        Pose fix = turret.consumePoseCorrection(); // camera-based odometry drift fix (off unless enabled)
        if (fix != null) {
            Pose p = follower.pose();
            follower.setPose(new Pose(p.x() + fix.x(), p.y() + fix.y(), p.heading()));
        }
        shooter.update();
        transfer.update();
        intake1.update();
        intake2.update();

        showPose(); // first line so it never scrolls off the Driver Station
        if (turret.isEncoderFault()) telemetry.addLine("!!! TURRET ENCODER FAULT: turret stopped !!!");
        if (turret.isAtLimit()) telemetry.addLine("!!! TURRET AT SOFT LIMIT !!!");
        telemetry.addData("state", state);
        telemetry.addData("shooter velocity", "%.1f", shooter.getActualVelocity());
        telemetry.addData("transfer", transfer.isOn());
        telemetry.addData("intake", intake1.isOn());
        telemetry.addData("turret mode", turret.getMode());
        telemetry.addData("turret goal (x, y)", "%.1f, %.1f", turret.getGoalX(), turret.getGoalY());
        telemetry.addData("camera tags used", turret.getCamera().getTagIds());
        telemetry.addData("turret angle / target (deg)", "%.1f / %.1f", turret.getAngleDeg(), turret.getTarget());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }
    }

    private void showPose() {
        Pose pose = follower.pose();
        telemetry.addData("odo x / y / heading (deg)", "%.2f / %.2f / %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    public Path intakeGarden() {
        return Paths.line(gardenStart, garden).constant(Math.toRadians(0))
                .with(Constants.foresightConfig.maxVelocityConstraint.at(25.0));
    }

    // Hold 360 for the first 35%, then turn to 270
    public Path driveToOtherSide() {
        return Paths.curve(garden, otherSideControl1, otherSideControl2, otherSide).heading(
                Interpolator.piecewise()
                        .until(0.35, Interpolator.constant(Math.toRadians(360)))
                        .until(1, Interpolator.linear(Math.toRadians(360), Math.toRadians(270))))
                .with(Constants.foresightConfig.maxVelocityConstraint.at(38.0));
    }

    public Path lineUpFlower() {
        return Paths.line(otherSide, flowerLineUp).constant(Math.toRadians(270));
    }

    public Path intakeFlower() {
        return Paths.line(flowerLineUp, flower).linear(Math.toRadians(270), Math.toRadians(270))
                .with(Constants.foresightConfig.maxVelocityConstraint.at(25.0));
    }

    public Path parkPath() {
        return Paths.line(flower, park).linear(Math.toRadians(270), Math.toRadians(270));
    }
}

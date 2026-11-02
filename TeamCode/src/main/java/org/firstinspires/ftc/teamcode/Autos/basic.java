package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.line;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.Vision.LimeLightCamera;
import org.firstinspires.ftc.teamcode.Vision.PedroAlign;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Basic State Test")
public class basic extends OpMode {
    private Follower follower;
    private PoseFactory poseFactory = PoseFactory.degrees();
    private Pose startPose = poseFactory.of(0, 0, 90);
    private Pose endPose = poseFactory.of(30, 0, 90);
    private Path startToPreAlign;
    private Path preAlignToPickUp;
    private Path pickupToPreAlign;
    private Path ParkPath;
    private DcMotor shooter;
    private DcMotor intake;
    private CRServo leftintake;
    private CRServo rightintake;
    private AutoState currentState;
    private LimeLightCamera camera;   // sees the tags
    private PedroAlign aligner;       // turns to the tag using Pedro
    private String alliance = "NOT SELECTED";
    private static final long SCORE_TIME = 2000;
    private static final long COLLECT_TIME = 1500;
    private static final long PARK_TIME = 27000;
    private static final double TILE = 24;            // inches per field tile
    private static final double PARK_LEFT = 1.5 * TILE; // park: strafe 1.5 tiles to the robot's left...
    private static final double PARK_FORWARD = 5;       // ...after going forward this many inches
    // park legs still to drive, followed one at a time so each move finishes before the next starts
    private final List<Path> parkLegs = new ArrayList<>();
    private long autoStartTime;
    private long stateStartTime;
    enum AutoState {
        DRIVETOSCORE,
        ALIGNTOGOAL,
        SCORE,
        DRIVETOPICKUP,
        COLLECT,
        PARK
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        buildPath();
        currentState = AutoState.PARK;
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intakemotor");
        rightintake = hardwareMap.get(CRServo.class, "rightintake");
        leftintake = hardwareMap.get(CRServo.class, "leftintake");
        rightintake.setDirection(DcMotorSimple.Direction.REVERSE);

        camera = new LimeLightCamera(hardwareMap);
        aligner = new PedroAlign(camera, follower);
    }

    // pick alliance during init so auto align only tracks our own HIVE
    @Override
    public void init_loop() {
        camera.update();

        if (gamepad1.xWasPressed()) {
            camera.setAlliance(LimeLightCamera.Alliance.BLUE);
            alliance = "BLUE";
        }
        if (gamepad1.bWasPressed()) {
            camera.setAlliance(LimeLightCamera.Alliance.RED);
            alliance = "RED";
        }

        telemetry.addLine("X = BLUE    B = RED");
        telemetry.addData("Alliance", alliance);
        camera.addDebugTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        autoStartTime = System.currentTimeMillis();
        // park only for now: strafe 2 tiles left and stop. The score/align/pickup states below are kept
        // for later; switch this back to AutoState.DRIVETOSCORE to run the full auto again
        enterState(AutoState.PARK);
    }

    @Override
    public void loop() {
        if (currentState != AutoState.PARK && System.currentTimeMillis() - autoStartTime >= PARK_TIME) {
            enterState(AutoState.PARK);
        }
        follower.update();
        camera.update();
        switch (currentState) {
            case DRIVETOSCORE:
                updateDriveToScore();
                break;
            case ALIGNTOGOAL:
                updateAligntoGoal();
                break;
            case SCORE:
                updateScore();
                break;
            case DRIVETOPICKUP:
                updateDriveToPickup();
                break;
            case COLLECT:
                updateCollect();
                break;
            case PARK:
                updatePark();
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.addData("State time", getStateTime());
        telemetry.addData("Busy?", follower.isBusy());
        telemetry.addData("Target seen", camera.hasTarget());
        telemetry.addData("tx", camera.getTx());
        aligner.addDebugTelemetry(telemetry);

        telemetry.update();
    }

    private void buildPath()
    {
        startToPreAlign = line(startPose, endPose).constant(endPose);
        preAlignToPickUp = line(endPose, startPose).constant(startPose); //test stuff go back and forth blah blha blahfdafd
    }

    private void enterState(AutoState newState) {
        currentState = newState;
        stateStartTime = System.currentTimeMillis();

        switch (newState) {
            case DRIVETOSCORE:
                startDrivetoScore();
                break;
            case ALIGNTOGOAL:
                startAlign();
                break;
            case SCORE:
                startScore();
                break;
            case DRIVETOPICKUP:
                startDriveToPickup();
                break;
            case COLLECT:
                startCollect();
                break;
            case PARK:
                // forward 5 in, then strafe 1.5 tiles left, relative to the robot, heading unchanged
                Pose p0 = follower.pose();
                Pose p1 = offset(p0, PARK_FORWARD, 0);
                Pose p2 = offset(p1, 0, PARK_LEFT);
                parkLegs.clear();
                parkLegs.add(line(p0, p1).constant(p1));
                parkLegs.add(line(p1, p2).constant(p2));
                follower.follow(parkLegs.remove(0));
                stopAll();
                break;
        }
    }


    //state-entry
    private void startDrivetoScore() {
        follower.follow(startToPreAlign);
    }
    private void startAlign() {
        aligner.start();
    }
    private void startScore() {
        shooter.setPower(0.5);
    }
    private void startDriveToPickup()
    {
        follower.follow(preAlignToPickUp);
    }
    private void startCollect() {
        intake.setPower(1);
        leftintake.setPower(1);
        rightintake.setPower(1);
    }
    private void stopAll() {
        stopScoring();
        intake.setPower(0);
        leftintake.setPower(0);
        rightintake.setPower(0);
    }

    //state-update
    private void updateDriveToScore() {
        if (!follower.isBusy()) {
            enterState(AutoState.ALIGNTOGOAL);
        }
    }
    private void updateAligntoGoal()
    {
        if (aligner.update()) {  // aligned, or gave up after the timeout
            enterState(AutoState.SCORE);
        }
    }
    private void updateScore() {
        if (getStateTime() >= SCORE_TIME) {
            stopScoring();
            enterState(AutoState.DRIVETOPICKUP);
        }
    }
    private void updateDriveToPickup() {
        if (!follower.isBusy()) {
            enterState(AutoState.COLLECT);
        }
    }
    private void updateCollect() {
        if (getStateTime() >= COLLECT_TIME) {
            intake.setPower(0);
            leftintake.setPower(0);
            rightintake.setPower(0);
            enterState(AutoState.DRIVETOSCORE);
        }
    }
    private void updatePark() {
        //everything already stopped on entry, just drive the remaining legs
        if (!follower.isBusy() && !parkLegs.isEmpty()) {
            follower.follow(parkLegs.remove(0));
        }
    }

    // pose moved relative to the robot: +forward along its heading, +left to its left (-sin h, cos h)
    private Pose offset(Pose p, double forward, double left) {
        double h = p.heading();
        return new Pose(p.x() + forward * Math.cos(h) - left * Math.sin(h),
                p.y() + forward * Math.sin(h) + left * Math.cos(h), h);
    }

    private void stopScoring() {
        shooter.setPower(0);
    }
    private long getStateTime() {
        return  System.currentTimeMillis() - stateStartTime;
    }
}
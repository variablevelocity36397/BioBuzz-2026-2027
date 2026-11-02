package org.firstinspires.ftc.teamcode.TeleOp;
import static com.pedropathing.api.Paths.line;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerLog;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Vision.AutoAlign;
import org.firstinspires.ftc.teamcode.Vision.LimeLightCamera;
import org.firstinspires.ftc.teamcode.Vision.LimeLightCamera.Alliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TeleOp", group = "TeleOp")

public class Teleop extends OpMode {

    // ---------- tuning ----------
    private static final double SHOOTER_VELOCITY = 2000;  // ticks/s, fixed for now (ShooterSpeed.fromTy() not hooked up yet)
    private static final double STICK_DEADBAND = 0.1;     // stick movement past this cancels auto modes
    private static final double SLOW_TRIGGER = 0.1;       // right trigger past this = slow mode
    private static final double SLOW_SPEED = 0.2;         // drive speed in slow mode
    private static final double TURN_SCALE = 0.8;         // right stick turn is scaled down a bit
    private static final double RESET_TRIGGER = 0.5;      // left trigger past this = reset pose
    private static final double TAG_LOST_TIMEOUT = 0.3;   // seconds without the tag before aligning gives up
    private static final double FIELD_SIZE = 144;         // inches, used to mirror red poses to blue
    private static final int RUMBLE_MS = 200;

    // ---------- mechanisms ----------
    private DcMotor intakemotor;   // intake motor
    private CRServo leftintake;
    private CRServo rightintake;
    private DcMotorEx shooter;
    private CRServo windmill;

    private boolean intakeOn = false;
    private boolean shooterOn = false;

    // one direction for the windmill, so dpad left and right can't get out of sync
    private enum WindmillDirection {
        OFF(0), FORWARD(1), REVERSE(-1);
        final double power;
        WindmillDirection(double power) { this.power = power; }
    }
    private WindmillDirection windmillDirection = WindmillDirection.OFF;

    // ---------- driving ----------
    private LimeLightCamera camera;   // sees the tags
    private AutoAlign aligner;        // works out the turn toward the tag, Pedro drives it
    private ElapsedTime tagLostTimer = new ElapsedTime();   // how long the tag has been out of view while aligning

    // who is driving the wheels right now. Only one at a time, so the robot can never be
    // aligning and driving to a shoot spot at once
    private enum DriveMode {
        MANUAL,            // driver sticks through Pedro's follower.manual()
        ALIGNING,          // AutoAlign's turn power through Pedro's follower.manual()
        DRIVING_TO_SHOOT   // Pedro follows the HIVE-avoiding route to a shoot spot
    }
    private DriveMode driveMode = DriveMode.MANUAL;

    private Follower follower;        // Pedro, the only thing that drives the wheels
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final HiveAvoider hiveAvoider = new HiveAvoider();   // routes around the HIVEs
    private int routeWaypoints = 0;   // corners in the last route, 0 = straight line
    private String routeText = "none";   // waypoints of the last route, for telemetry
    // legs still to drive. Followed one at a time so Pedro brakes at each corner instead of
    // carrying speed into it and swinging wide
    private final List<Path> pendingLegs = new ArrayList<>();
    private boolean leftTriggerWasDown = false;
    // false until the driver resets the pose with the left trigger. The pose set in init is a guess
    // (after auto the robot is somewhere else), so drive to shoot is blocked until then
    private boolean poseReset = false;

    // TODO tune on field. Only the red spots are written down, blue is red mirrored (see mirrorForBlue)
    private final Pose startPose = poseFactory.of(9, 9, 90);          // bottom left corner, facing up the field
    private final Pose redShootUp = poseFactory.of(60, 132, 90);      // dpad up
    private final Pose redShootDown = poseFactory.of(56, 8, 270);     // dpad down

    private Alliance alliance = null;   // null until picked in init

    @Override
    public void init() {

        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        leftintake = hardwareMap.get(CRServo.class, "leftintake");
        rightintake = hardwareMap.get(CRServo.class, "rightintake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // setVelocity needs this
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // flywheel coasts down instead of braking
        windmill = hardwareMap.get(CRServo.class, "windmill");
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        rightintake.setDirection(DcMotorSimple.Direction.REVERSE);

        camera = new LimeLightCamera(hardwareMap);
        aligner = new AutoAlign(camera);   // math only, the drive motors belong to Pedro

        follower = Constants.create(hardwareMap);   // sets up the drive motors and their directions
        follower.setPose(startPose);
    }

    // pick alliance during init so auto align only tracks our own HIVE
    @Override
    public void init_loop() {
        camera.update();

        if (gamepad1.xWasPressed()) {
            setAlliance(Alliance.BLUE);
        }
        if (gamepad1.bWasPressed()) {
            setAlliance(Alliance.RED);
        }

        telemetry.addLine("--------  SELECT ALLIANCE --------");
        telemetry.addLine("X = BLUE    B = RED");
        telemetry.addData("Alliance", allianceText());

        addControlsTelemetry();

        telemetry.addLine("--------  LIMELIGHT DEBUG --------");
        camera.addDebugTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void loop() {
        camera.update();   // read the Limelight once per loop

        double y = -gamepad1.left_stick_y;     // forward / back
        double x = gamepad1.left_stick_x;      // strafe
        double rx = gamepad1.right_stick_x * TURN_SCALE;   // turn
        double speed = gamepad1.right_trigger > SLOW_TRIGGER ? SLOW_SPEED : 1;

        handleDriveButtons();
        updateDrive(y, x, rx, speed);
        handleMechanismButtons();

        addControlsTelemetry();
        addStatusTelemetry(speed);
        telemetry.update();
    }

    // ---------- driving ----------

    // buttons that change the drive mode
    private void handleDriveButtons() {
        // 1. read every input once per loop. The xxxWasPressed() calls remember a press until they're
        //    called, so they must all run every loop, or a press skipped by the else-if chain below
        //    would fire loops later (e.g. X tapped while steering would start aligning on release)
        boolean stickMoved = Math.abs(gamepad1.left_stick_x) > STICK_DEADBAND || Math.abs(gamepad1.left_stick_y) > STICK_DEADBAND
                || Math.abs(gamepad1.right_stick_x) > STICK_DEADBAND || Math.abs(gamepad1.right_stick_y) > STICK_DEADBAND;
        boolean leftTriggerDown = gamepad1.left_trigger > RESET_TRIGGER;
        boolean resetPressed = leftTriggerDown && !leftTriggerWasDown;
        leftTriggerWasDown = leftTriggerDown;
        boolean dpadUpPressed = gamepad1.dpadUpWasPressed();
        boolean dpadDownPressed = gamepad1.dpadDownWasPressed();
        boolean xPressed = gamepad1.xWasPressed();

        // 2. decide, highest priority first. Exactly one branch runs per loop:
        //    manual (sticks) > reset pose (left trigger) > drive to shoot (dpad) > aligning (x)
        if (stickMoved) {
            // any joystick movement cancels auto align and driving to a shoot spot
            setDriveMode(DriveMode.MANUAL);   // the MANUAL case takes the wheels back this loop
        } else if (resetPressed) {
            // robot is sitting in the 9,9 corner facing up the field, reset the pose there to fix drift
            follower.setPose(startPose);
            poseReset = true;   // pose is trusted now, drive to shoot is allowed
            setDriveMode(DriveMode.MANUAL);
            feedbackOk();
        } else if (dpadUpPressed || dpadDownPressed) {
            // drive to a shoot spot with Pedro, overrides aligning
            startDriveToShoot(dpadUpPressed ? redShootUp : redShootDown);
        } else if (xPressed) {
            // auto align to the apriltag, but never interrupt a drive to a shoot spot
            if (driveMode == DriveMode.DRIVING_TO_SHOOT || !camera.hasTarget()) {
                feedbackReject();    // busy driving to a shoot spot, or no tag: nothing happens
            } else {
                setDriveMode(DriveMode.ALIGNING);
                feedbackOk();        // tag seen, aligning
            }
        }
    }

    // redSpot is the red alliance's pose, mirrored here if we're blue
    private void startDriveToShoot(Pose redSpot) {
        if (alliance == null) {
            feedbackReject();    // no alliance picked, don't know where to go
            return;
        }
        if (!poseReset) {
            feedbackReject();    // pose not reset yet, we don't know where we are, so don't plan a route
            return;
        }
        Pose target = alliance == Alliance.BLUE ? mirrorForBlue(redSpot) : redSpot;
        Pose current = follower.pose();
        List<Pose> route = hiveAvoider.route(current, target);
        if (route == null) {
            feedbackReject();    // no way around the HIVE, stay put
            return;
        }

        // straight legs, each turning from its start heading to its end heading,
        // so the turn is spread over the whole drive
        pendingLegs.clear();
        StringBuilder text = new StringBuilder();
        Pose from = current;
        for (Pose to : route) {
            pendingLegs.add(line(from, to).linear(from, to));
            text.append(String.format("(%.0f, %.0f) ", to.x(), to.y()));
            from = to;
        }
        follower.follow(pendingLegs.remove(0));
        routeWaypoints = route.size() - 1;
        routeText = text.toString();
        setDriveMode(DriveMode.DRIVING_TO_SHOOT);
        feedbackOk();
    }

    // Pedro drives the wheels in every mode, so its motor cache always matches the hardware
    private void updateDrive(double y, double x, double rx, double speed) {
        switch (driveMode) {
            case DRIVING_TO_SHOOT:
                if (!follower.isBusy()) {
                    if (pendingLegs.isEmpty()) {   // got there, give control back
                        setDriveMode(DriveMode.MANUAL);
                        driveManual(y, x, rx, speed);
                        break;
                    }
                    follower.follow(pendingLegs.remove(0));   // reached a corner, start the next leg
                }
                follower.update();   // Pedro follows the path
                break;

            case ALIGNING:
                // tag gone too long while aligning: give up and give control back
                if (camera.hasTarget()) {
                    tagLostTimer.reset();
                } else if (tagLostTimer.seconds() > TAG_LOST_TIMEOUT) {
                    setDriveMode(DriveMode.MANUAL);
                    feedbackReject();
                    driveManual(y, x, rx, speed);
                    break;
                }
                if (aligner.isAligned()) {   // lined up, stop turning and give control back
                    setDriveMode(DriveMode.MANUAL);
                    driveManual(0, 0, 0, 1);
                    break;
                }
                driveManual(0, 0, aligner.turnPower(), 1);   // turn in place, full speed (AutoAlign already caps it)
                break;

            case MANUAL:
                driveManual(y, x, rx, speed);
                break;
        }
    }

    // the one place the drive mode changes, so the cleanup for each mode can't be forgotten
    private void setDriveMode(DriveMode newMode) {
        if (newMode != DriveMode.DRIVING_TO_SHOOT) {
            pendingLegs.clear();   // drop any route legs we didn't finish
        }
        if (newMode == DriveMode.ALIGNING) {
            tagLostTimer.reset();
        }
        driveMode = newMode;
    }

    // positive rx turns right, same as the right stick
    private void driveManual(double y, double x, double rx, double speed) {
        // Pedro's mecanum: fl = forward - strafe - turn, ours was fl = y + x + rx, so flip strafe and turn
        follower.manual(y * speed, -x * speed, -rx * speed);
        follower.update();
    }

    // ---------- alliance ----------

    private void setAlliance(Alliance newAlliance) {
        alliance = newAlliance;
        camera.setAlliance(newAlliance);
    }

    private String allianceText() {
        return alliance == null ? "NOT SELECTED" : alliance.toString();
    }

    // blue's field is red's mirrored left to right: x -> 144 - x, heading -> 180 - heading
    private Pose mirrorForBlue(Pose red) {
        double heading = Math.PI - red.heading();
        if (heading < 0) {
            heading += 2 * Math.PI;   // keep it 0..360 like the red poses
        }
        return new Pose(FIELD_SIZE - red.x(), red.y(), heading);
    }

    // ---------- mechanisms ----------

    private void handleMechanismButtons() {
        if (gamepad1.aWasPressed()) {  // a to toggle the intake
            intakeOn = !intakeOn;
            double power = intakeOn ? 1 : 0;
            intakemotor.setPower(power);
            leftintake.setPower(power);
            rightintake.setPower(power);
        }
        if (gamepad1.bWasPressed()) {  // b to toggle the shooter
            shooterOn = !shooterOn;
            if (shooterOn) {
                shooter.setVelocity(SHOOTER_VELOCITY);
            } else {
                shooter.setPower(0);
            }
        }
        // dpad left/right: run the windmill that way, press the same one again to stop
        if (gamepad1.dpadLeftWasPressed()) {
            toggleWindmill(WindmillDirection.FORWARD);
        }
        if (gamepad1.dpadRightWasPressed()) {
            toggleWindmill(WindmillDirection.REVERSE);
        }
    }

    private void toggleWindmill(WindmillDirection direction) {
        windmillDirection = windmillDirection == direction ? WindmillDirection.OFF : direction;
        windmill.setPower(windmillDirection.power);
    }

    // ---------- feedback ----------

    private void feedbackOk() {
        gamepad1.rumble(RUMBLE_MS);       // one short rumble = doing it
    }

    private void feedbackReject() {
        gamepad1.rumbleBlips(3);          // three quick blips = can't, nothing happens
    }

    // ---------- telemetry ----------

    private void addControlsTelemetry() {
        telemetry.addLine("--------  GAMEPAD CONTROLS --------");
        telemetry.addLine("Left stick: drive / strafe");
        telemetry.addLine("Right stick X: turn");
        telemetry.addLine("Right trigger: slow mode");
        telemetry.addLine("A: toggle intake");
        telemetry.addLine("B: toggle shooter");
        telemetry.addLine("X: auto align (move any stick to cancel)");
        telemetry.addLine("Dpad up/down: drive to shoot spot (needs a left trigger reset first, move any stick to cancel)");
        telemetry.addLine("Dpad left/right: windmill forward/reverse (same button again stops)");
        telemetry.addLine("Left trigger: reset pose (robot at 9,9 facing up field)");
    }

    private void addStatusTelemetry(double speed) {
        telemetry.addLine("--------  STATUS --------");
        if (!poseReset) {
            telemetry.addLine("!! POSE NOT RESET: park in the 9,9 corner and press left trigger to enable dpad !!");
        }
        telemetry.addData("Intake on", intakeOn);
        telemetry.addData("Shooter on", shooterOn);
        telemetry.addData("Shooter target", SHOOTER_VELOCITY);
        telemetry.addData("Shooter velocity", shooter.getVelocity());
        telemetry.addData("Windmill", windmillDirection);
        telemetry.addData("Slow mode", speed < 1);
        telemetry.addData("Alliance", allianceText());
        telemetry.addData("Target seen", camera.hasTarget());
        telemetry.addData("Drive mode", driveMode);
        telemetry.addData("Route corners", routeWaypoints);
        telemetry.addData("Route", routeText);
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("tx", camera.getTx());
        telemetry.addData("ty", camera.getTargetTy());

        telemetry.addLine("--------  PEDRO DEBUG --------");
        telemetry.addData("Mode", follower.mode());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Path index", follower.pathIndex());
        FollowerLog log = follower.debug();
        telemetry.addData("Drivetrain", log.drivetrain());
        telemetry.addData("Algorithm", log.algorithm());

        telemetry.addLine("--------  LIMELIGHT DEBUG --------");
        camera.addDebugTelemetry(telemetry);
    }
}

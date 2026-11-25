package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsytems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.TurretTrack;

/** Odometry turret tracking. Drive with gamepad1 or push the robot by hand; Pinpoint tracks either way. */
@TeleOp(name = "Test Turret Track", group = "Test")
public class TestTurretTrack extends OpMode {
    private PinpointLocalizer localizer;
    private TurretTrack turret;
    private MecanumDrive drive;
    private boolean tracking;
    private final double[] steps = {0.001, 0.0001};
    private int stepIndex;

    @Override
    public void init() {
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        turret = new TurretTrack(hardwareMap);
        drive = new MecanumDrive(hardwareMap);
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
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * 0.8, 1.0);
        if (gamepad1.aWasPressed()) tracking = !tracking;
        // Live tuning, resets when the OpMode stops: copy good values into TurretTrack.KP / KD
        if (gamepad1.bWasPressed()) stepIndex = (stepIndex + 1) % steps.length;
        double step = steps[stepIndex];
        if (gamepad1.dpadUpWasPressed()) turret.setKd(turret.getKd() + step);
        if (gamepad1.dpadDownWasPressed()) turret.setKd(turret.getKd() - step);
        if (gamepad1.dpadRightWasPressed()) turret.setKp(turret.getKp() + step);
        if (gamepad1.dpadLeftWasPressed()) turret.setKp(turret.getKp() - step);

        localizer.update();
        Pose pose = localizer.state().pose();
        turret.update(pose, tracking);

        if (turret.isAtLimit()) telemetry.addLine("!!! TURRET AT SOFT LIMIT !!!");
        telemetry.addData("Alliance", turret.getAlliance());
        telemetry.addData("A (Cross) - tracking", tracking ? "ON" : "OFF (hold forward)");
        telemetry.addData("Dpad Up/Down - KD", "%.4f", turret.getKd());
        telemetry.addData("Dpad Right/Left - KP", "%.4f", turret.getKp());
        telemetry.addData("B (Circle) - step size", "%.4f", step);
        telemetry.addData("Min power", "%.3f", TurretTrack.MIN_POWER);
        telemetry.addData("Pose (x, y, heading deg)", "%.1f, %.1f, %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Goal (x, y)", "%.1f, %.1f", turret.getGoalX(), turret.getGoalY());
        telemetry.addData("Aim x", "%.1f", turret.getAimX());
        telemetry.addData("Field angle to goal (deg)", "%.1f", turret.getFieldAngleDeg());
        telemetry.addData("Turret angle (deg)", "%.1f", turret.getAngleDeg());
        telemetry.addData("Turret target (deg)", "%.1f", turret.getTarget());
        telemetry.addData("Error (deg)", "%.1f", turret.getError());
        telemetry.addData("Servo power", "%.3f", turret.getPower());
        telemetry.addData("Soft limits (deg)", "%.0f to %.0f", TurretTrack.MIN_DEGREE, TurretTrack.MAX_DEGREE);
        telemetry.update();
    }
}

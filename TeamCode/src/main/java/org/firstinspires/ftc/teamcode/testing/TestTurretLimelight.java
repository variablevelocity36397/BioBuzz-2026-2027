package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsytems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.TurretLimelight;
import org.firstinspires.ftc.teamcode.subsytems.TurretTrack;
import org.firstinspires.ftc.teamcode.vision.HiveTagCamera;

/** Limelight turret tracking with odometry fallback. Start on the alliance start pose. Drive with gamepad1. */
@TeleOp(name = "Test Turret Limelight", group = "Test")
public class TestTurretLimelight extends OpMode {
    private PinpointLocalizer localizer;
    private TurretLimelight turret;
    private MecanumDrive drive;
    private boolean tracking;

    @Override
    public void init() {
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        turret = new TurretLimelight(hardwareMap);
        drive = new MecanumDrive(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) turret.setAlliance(TurretTrack.Alliance.BLUE);
        if (gamepad1.bWasPressed()) turret.setAlliance(TurretTrack.Alliance.RED);
        telemetry.addData("ALLIANCE", turret.getAlliance());
        telemetry.addLine("X (Square) = BLUE, B (Circle) = RED");
        telemetry.addLine("Point the turret straight forward by hand before pressing play.");
        telemetry.addData("Start pose (odometry fallback)", turret.getAlliance().start);
        telemetry.update();
    }

    @Override
    public void start() {
        localizer.setPose(turret.getAlliance().start); // odometry fallback needs the real start pose
        turret.zero();
    }

    @Override
    public void loop() {
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * 0.8, 1.0);
        if (gamepad1.aWasPressed()) tracking = !tracking;
        // Live tuning, resets when the OpMode stops: copy a good value into TurretLimelight.DEADBAND_DEG
        if (gamepad1.dpadUpWasPressed()) turret.setDeadband(turret.getDeadband() + 0.1);
        if (gamepad1.dpadDownWasPressed()) turret.setDeadband(turret.getDeadband() - 0.1);
        localizer.update();
        Pose pose = localizer.state().pose();
        turret.update(pose, tracking);

        HiveTagCamera cam = turret.getCamera();
        if (turret.isEncoderFault()) telemetry.addLine("!!! TURRET ENCODER FAULT: turret stopped, restart OpMode !!!");
        if (turret.isAtLimit()) telemetry.addLine("!!! TURRET AT SOFT LIMIT !!!");
        telemetry.addData("A (Cross) - tracking", tracking ? "ON" : "OFF (hold forward)");
        telemetry.addData("Mode", turret.getMode());
        telemetry.addData("Alliance", turret.getAlliance());
        telemetry.addData("Dpad Up/Down - camera deadband (deg)", "%.1f", turret.getDeadband());
        telemetry.addData("Tags (count)", "%s(%d)", cam.getTagIds(), cam.getTagCount());
        telemetry.addData("Avg tx / ty (deg)", "%.1f / %.1f", cam.getTx(), cam.getTy());
        telemetry.addData("Area", "%.2f", cam.getTargetArea());
        telemetry.addData("Shooter offset (deg right)", "%.1f", turret.getShooterOffsetDeg());
        telemetry.addData("Frame age / latency (ms)", "%d / %.0f", cam.getFrameAgeMs(), cam.getLatencyMs());
        telemetry.addData("Turret angle (deg)", "%.1f", turret.getAngleDeg());
        telemetry.addData("Turret target (deg)", "%.1f", turret.getTarget());
        telemetry.addData("Error (deg)", "%.1f", turret.getError());
        telemetry.addData("Servo power", "%.3f", turret.getPower());
        telemetry.addData("Soft limits (deg)", "%.0f to %.0f", TurretTrack.MIN_DEGREE, TurretTrack.MAX_DEGREE);
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsytems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsytems.TurretLimelight;
import org.firstinspires.ftc.teamcode.subsytems.TurretTrack;
import org.firstinspires.ftc.teamcode.vision.HiveTagCamera;

/**
 * Checks the camera's field pose against odometry before trusting it. Start on the alliance start pose with the
 * turret pointed forward. Drive with gamepad1. Park somewhere with the tags in view: "Vision minus odometry" should
 * sit near 0 in x, y and heading. If x and y are off by a fixed swap or sign, fix VisionPose.FIELD_ROTATION_DEG; if
 * they change with the turret angle, fix VisionPose.CAMERA_*_OF_PIVOT_IN. Y turns the odometry correction on/off.
 */
@TeleOp(name = "Test Vision Pose", group = "Test")
public class TestVisionPose extends OpMode {
    private PinpointLocalizer localizer;
    private TurretLimelight turret;
    private MecanumDrive drive;

    @Override
    public void init() {
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        turret = new TurretLimelight(hardwareMap);
        drive = new MecanumDrive(hardwareMap);
        turret.resetEncoder();
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
        turret.poseWasReset();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * 0.8, 1.0);
        if (gamepad1.yWasPressed()) turret.setPoseCorrection(!turret.isPoseCorrection());
        localizer.update();
        Pose pose = localizer.state().pose();
        turret.update(pose, true);
        Pose fix = turret.consumePoseCorrection();
        if (fix != null) {
            pose = new Pose(pose.x() + fix.x(), pose.y() + fix.y(), pose.heading());
            localizer.setPose(pose);
        }

        HiveTagCamera cam = turret.getCamera();
        Pose v = turret.getVisionPose();
        telemetry.addData("Y (Triangle) - odometry correction", turret.isPoseCorrection() ? "ON" : "OFF");
        telemetry.addData("Odometry x / y / heading (deg)", "%.1f / %.1f / %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Limelight field pose (m) x / y / yaw (deg)", cam.hasFieldPose()
                ? String.format("%.3f / %.3f / %.1f", cam.getFieldX(), cam.getFieldY(), cam.getFieldYawDeg()) : "none");
        telemetry.addData("Field-pose tags", cam.getFieldTagCount());
        if (v == null) {
            telemetry.addData("Vision robot pose", "no usable frame (needs %d+ tags, fresh)", TurretLimelight.MIN_POSE_TAGS);
        } else {
            telemetry.addData("Vision robot pose x / y / heading (deg)", "%.1f / %.1f / %.1f", v.x(), v.y(), Math.toDegrees(v.heading()));
            telemetry.addData("Vision minus odometry (x / y in, heading deg)", "%.1f / %.1f / %.1f",
                    turret.getVisionDx(), turret.getVisionDy(), turret.getVisionHeadingError());
            telemetry.addData("Passed checks", turret.isVisionAccepted());
        }
        telemetry.addData("Turret mode", turret.getMode());
        telemetry.addData("Turret angle (deg)", "%.1f", turret.getAngleDeg());
        telemetry.update();
    }
}

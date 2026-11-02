package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Vision.AutoAlign;
import org.firstinspires.ftc.teamcode.Vision.LimeLightCamera;
import org.firstinspires.ftc.teamcode.Vision.ShooterSpeed;

// For filling in ShooterSpeed.TABLE: set the shooter speed by hand, read ty off the screen
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends OpMode {

    private DcMotor intakemotor;
    private CRServo leftintake;
    private CRServo rightintake;
    private DcMotor frwheel;
    private DcMotor flwheel;
    private DcMotor brwheel;
    private DcMotor blwheel;
    private DcMotorEx shooter;
    private CRServo windmill;

    private LimeLightCamera camera;
    private AutoAlign aligner;
    private boolean aligning = false;

    private double velocity = 2100;   // ticks/s, changed with the dpad
    private boolean shooterOn = false;
    private boolean intakeOn = false;
    private boolean windmillOn = false;

    private String alliance = "NOT SELECTED";

    @Override
    public void init() {
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        leftintake = hardwareMap.get(CRServo.class, "leftintake");
        rightintake = hardwareMap.get(CRServo.class, "rightintake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        windmill = hardwareMap.get(CRServo.class, "windmill");
        frwheel = hardwareMap.get(DcMotor.class, "fr");
        flwheel = hardwareMap.get(DcMotor.class, "fl");
        brwheel = hardwareMap.get(DcMotor.class, "br");
        blwheel = hardwareMap.get(DcMotor.class, "bl");
        flwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        blwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        rightintake.setDirection(DcMotorSimple.Direction.REVERSE);

        camera = new LimeLightCamera(hardwareMap);
        aligner = new AutoAlign(hardwareMap, camera);
    }

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
        telemetry.update();
    }

    @Override
    public void loop() {
        camera.update();

        // dpad up/down = +/-50, dpad right/left = +/-10
        if (gamepad1.dpadUpWasPressed()) velocity += 25;
        if (gamepad1.dpadDownWasPressed()) velocity -= 25;
        if (gamepad1.dpadRightWasPressed()) velocity += 5;
        if (gamepad1.dpadLeftWasPressed()) velocity -= 5;
        velocity = Math.max(0, velocity);

        if (gamepad1.bWasPressed()) shooterOn = !shooterOn;
        if (shooterOn) {
            shooter.setVelocity(velocity);
        } else {
            shooter.setPower(0);
        }

        if (gamepad1.aWasPressed()) {
            intakeOn = !intakeOn;
            double p = intakeOn ? 1 : 0;
            intakemotor.setPower(p);
            leftintake.setPower(p);
            rightintake.setPower(p);
        }
        if (gamepad1.yWasPressed()) {
            windmillOn = !windmillOn;
            windmill.setPower(windmillOn ? 1 : 0);
        }

        // X = auto align, any stick cancels
        if (gamepad1.xWasPressed() && camera.hasTarget()) aligning = true;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 0.8;
        if (Math.abs(x) > 0.1 || Math.abs(y) > 0.1 || Math.abs(rx) > 0.1) aligning = false;

        if (aligning) {
            if (aligner.align() || !camera.hasTarget()) aligning = false;
        } else {
            double speed = gamepad1.right_trigger > 0.1 ? 0.2 : 1;
            flwheel.setPower((y + x + rx) * speed);
            blwheel.setPower((y - x + rx) * speed);
            frwheel.setPower((y - x - rx) * speed);
            brwheel.setPower((y + x - rx) * speed);
        }

        telemetry.addLine("--------  WRITE THESE DOWN --------");
        telemetry.addData("ty", camera.hasTarget() ? String.format("%.2f", camera.getTargetTy()) : "NO TAG");
        telemetry.addData("Velocity (set)", velocity);
        telemetry.addLine("--------  INFO --------");
        telemetry.addData("Velocity (actual)", "%.0f", shooter.getVelocity());
        telemetry.addData("Table would pick", camera.hasTarget()
                ? String.format("%.0f", ShooterSpeed.fromTy(camera.getTargetTy())) : "-");
        telemetry.addData("Shooter on", shooterOn);
        telemetry.addData("Aligning", aligning);
        telemetry.addData("tx", camera.getTx());
        telemetry.addLine("Dpad up/down +/-50, right/left +/-10");
        telemetry.addLine("B shooter, A intake, Y windmill, X align");
        telemetry.update();
    }
}

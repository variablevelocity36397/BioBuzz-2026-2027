package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Vision.AutoAlign;
import org.firstinspires.ftc.teamcode.Vision.LimeLightCamera;

@TeleOp(name = "TeleOp", group = "TeleOp")

public class Teleop extends OpMode {

    private DcMotor intakemotor;   // intake motor
    private CRServo leftintake;
    private CRServo rightintake;
    private DcMotor frwheel; // front right wheel
    private DcMotor flwheel; // front left wheel
    private DcMotor brwheel; // back right wheel
    private DcMotor blwheel; // back left wheel
    private DcMotor shooter;
    private CRServo windmill;

    private LimeLightCamera camera;   // sees the tags
    private AutoAlign aligner;        // turns the robot
    private boolean aligning = false;   // field so it remembers between loops
    private ElapsedTime tagLostTimer = new ElapsedTime();   // how long the tag has been out of view while aligning

    private boolean flag = true;
    private boolean flag2 = true;
    private boolean flag3 = true;
    private boolean flag4 = true;

    private String alliance = "NOT SELECTED";

    @Override
    public void init() {

        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        leftintake = hardwareMap.get(CRServo.class, "leftintake");
        rightintake = hardwareMap.get(CRServo.class, "rightintake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
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

        telemetry.addLine("--------  SELECT ALLIANCE --------");
        telemetry.addLine("X = BLUE    B = RED");
        telemetry.addData("Alliance", alliance);

        telemetry.addLine("--------  GAMEPAD CONTROLS --------");
        telemetry.addLine("Left stick: drive / strafe");
        telemetry.addLine("Right stick X: turn");
        telemetry.addLine("Right trigger: slow mode");
        telemetry.addLine("A: toggle intake");
        telemetry.addLine("B: toggle shooter");
        telemetry.addLine("Y: toggle windmill");
        telemetry.addLine("X: auto align (move any stick to cancel)");

        telemetry.addLine("--------  LIMELIGHT DEBUG --------");
        camera.addDebugTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void loop() {
        camera.update();   // read the Limelight once per loop

        double y = -gamepad1.left_stick_y ;       // left joystick control forward/back movement
        double x = gamepad1.left_stick_x ;     // right joystick control right/left turning
        double rx = gamepad1.right_stick_x * 0.8;
        double speed = 1;
        if (gamepad1.right_trigger > 0.1) {
            speed = 0.2;
        }

        if (gamepad1.xWasPressed()) {  // click x to auto align to the apriltag
            if (camera.hasTarget()) {
                aligning = true;
                tagLostTimer.reset();
                gamepad1.rumble(200);       // one short rumble = tag seen, aligning
            } else {
                gamepad1.rumbleBlips(3);    // three quick blips = no tag, nothing happens
            }
        }
        // any joystick movement cancels auto align
        if (Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1
                || Math.abs(gamepad1.right_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_y) > 0.1) {
            aligning = false;
        }

        // tag gone for more than 0.3 s while aligning: give up and give control back
        if (aligning) {
            if (camera.hasTarget()) {
                tagLostTimer.reset();
            } else if (tagLostTimer.seconds() > 0.3) {
                aligning = false;
                gamepad1.rumbleBlips(3);
            }
        }

        if (aligning) {
            if (aligner.align()) {  // align() returns true once lined up
                aligning = false;
            }
        } else {
            double leftFrontPower = (y + x + rx) ;
            double leftBackPower = (y - x + rx) ;
            double rightFrontPower = (y - x - rx) ;
            double rightBackPower = (y + x - rx) ;

            flwheel.setPower(leftFrontPower * speed);
            blwheel.setPower(leftBackPower * speed);
            frwheel.setPower(rightFrontPower * speed);
            brwheel.setPower(rightBackPower * speed);
        }

        if (gamepad1.aWasPressed()) {  // click the button a to toggle the intake
            if (flag) {
                intakemotor.setPower(1);
                leftintake.setPower(1);
                rightintake.setPower(1);
            } else {
                intakemotor.setPower(0);
                leftintake.setPower(0);
                rightintake.setPower(0);
            }
            flag = !flag;
        }
        if (gamepad1.bWasPressed()) {
            if (flag2) {
                shooter.setPower(1);
            } else {
                shooter.setPower(0);
            }
            flag2 = !flag2;
        }
        if (gamepad1.yWasPressed()) {
            if (flag3) {
                windmill.setPower(1);
            } else {
                windmill.setPower(0);
            }
            flag3 = !flag3;
        }





        telemetry.addLine("--------  GAMEPAD CONTROLS --------");
        telemetry.addLine("Left stick: drive / strafe");
        telemetry.addLine("Right stick X: turn");
        telemetry.addLine("Right trigger: slow mode");
        telemetry.addLine("A: toggle intake");
        telemetry.addLine("B: toggle shooter");
        telemetry.addLine("Y: toggle windmill");
        telemetry.addLine("X: auto align (move any stick to cancel)");

        telemetry.addLine("--------  STATUS --------");
        telemetry.addData("Intake on", !flag);   // flag flips after each press, so !flag = currently on
        telemetry.addData("Shooter on", !flag2);
        telemetry.addData("Windmill on", !flag3);
        telemetry.addData("Slow mode", speed < 1);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Target seen", camera.hasTarget());
        telemetry.addData("Aligning", aligning);
        telemetry.addData("tx", camera.getTx());

        telemetry.addLine("--------  LIMELIGHT DEBUG --------");
        camera.addDebugTelemetry(telemetry);
        telemetry.update();
    }
}

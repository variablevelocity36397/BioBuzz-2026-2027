package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {

    private DcMotor intakemotor;   // intake motor
    private CRServo leftintake;
    private CRServo rightintake;
    private DcMotor frwheel; // front right wheel
    private DcMotor flwheel; // front left wheel
    private DcMotor brwheel; // back right wheel
    private DcMotor blwheel; // back left wheel
    private DcMotor shooter;
    private CRServo windmill;

    public void runOpMode() {

        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        frwheel = hardwareMap.get(DcMotor.class, "fr");
        flwheel = hardwareMap.get(DcMotor.class, "fl");
        brwheel = hardwareMap.get(DcMotor.class, "br");
        blwheel = hardwareMap.get(DcMotor.class, "bl");
        flwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        blwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("--------  GAMEPAD CONTROLS --------");
        telemetry.addLine("Press A to toggle intake");


        boolean flag = true;
        boolean flag2 = true;
        boolean flag3 = true;
        boolean flag4 = true;



        waitForStart();


        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y ;       // left joystick control forward/back movement
            double x = gamepad1.left_stick_x ;     // right joystick control right/left turning
            double rx = gamepad1.right_stick_x * 0.8;
            double speed = 1;
            if (gamepad1.right_trigger > 0.1) {
                speed = 0.2;
            }

            double leftFrontPower = (y + x + rx) ;
            double leftBackPower = (y - x + rx) ;
            double rightFrontPower = (y - x - rx) ;
            double rightBackPower = (y + x - rx) ;

            flwheel.setPower(leftFrontPower * speed);
            blwheel.setPower(leftBackPower * speed);
            frwheel.setPower(rightFrontPower * speed);
            brwheel.setPower(rightBackPower * speed);

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
                    shooter.setPower(0.5);
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
            telemetry.addLine("Press A to toggle intake");
            telemetry.update();
        }
    }
}
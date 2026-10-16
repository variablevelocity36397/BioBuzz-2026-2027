package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class TestShootingIntake extends LinearOpMode {
    private DcMotor shooter;
    private DcMotor intake;

    @Override
    public void runOpMode() {
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        boolean flag1 = true;
        boolean flag2 = true;

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.aWasPressed()) {
                if (flag1) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }
                flag1 = !flag1;
            }
            if (gamepad1.bWasPressed()) {
                if (flag2) {
                    shooter.setPower(0.85);
                } else {
                    shooter.setPower(0);
                }
                flag2 = !flag2;
            }
        }
    }
}

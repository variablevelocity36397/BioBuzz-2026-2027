package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Robot-centric mecanum drive with normalized powers. Teleop/tests only: in autos Pedro drives the wheels. */
public class MecanumDrive {
    private final DcMotor fl, fr, bl, br;

    public MecanumDrive(HardwareMap hw) {
        fl = hw.get(DcMotor.class, "flwheel");
        fr = hw.get(DcMotor.class, "frwheel");
        bl = hw.get(DcMotor.class, "blwheel");
        br = hw.get(DcMotor.class, "brwheel");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double y, double x, double rx, double speed) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        fl.setPower((y + x + rx) / denominator * speed);
        bl.setPower((y - x + rx) / denominator * speed);
        fr.setPower((y - x - rx) / denominator * speed);
        br.setPower((y + x - rx) / denominator * speed);
    }
}

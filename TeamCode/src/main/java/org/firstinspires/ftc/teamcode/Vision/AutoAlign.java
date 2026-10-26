package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Turns the robot to face our tag, LimeLightCamera does the seeing
public class AutoAlign {

    private DcMotor frwheel;
    private DcMotor flwheel;
    private DcMotor blwheel;
    private DcMotor brwheel;
    private LimeLightCamera camera;

    // Limelight 3A sees about +/-27 deg horizontally, so 0.5 / 25 deg ~= 0.02
    private static final double kP = 0.02;
    private static final double MAX_TURN = 0.5;   // fastest it's allowed to turn
    private static final double MIN_TURN = 0.05;  // smallest push that still moves the robot
    private static final double TOLERANCE = 1.0;  // degrees of tx considered aligned

    public AutoAlign(HardwareMap hardwareMap, LimeLightCamera camera) {
        this.camera = camera;

        frwheel = hardwareMap.get(DcMotor.class, "fr");
        flwheel = hardwareMap.get(DcMotor.class, "fl");
        brwheel = hardwareMap.get(DcMotor.class, "br");
        blwheel = hardwareMap.get(DcMotor.class, "bl");
        flwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        blwheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Positive power turns right, matching rx in Teleop
    private void turn(double power) {
        flwheel.setPower(power);
        blwheel.setPower(power);
        frwheel.setPower(-power);
        brwheel.setPower(-power);
    }

    public void stop() {
        turn(0);
    }

    // Call once per loop. Returns true when facing the tag.
    public boolean align() {
        double tx = camera.getTargetTx();
        if (Double.isNaN(tx)) {
            stop();
            return false;
        }

        if (Math.abs(tx) < TOLERANCE) {
            stop();
            return true;
        }

        double power = kP * tx;
        if (Math.abs(power) < MIN_TURN) {
            power = Math.copySign(MIN_TURN, power);
        }
        power = Math.max(-MAX_TURN, Math.min(MAX_TURN, power));

        turn(power);
        return false;
    }
}

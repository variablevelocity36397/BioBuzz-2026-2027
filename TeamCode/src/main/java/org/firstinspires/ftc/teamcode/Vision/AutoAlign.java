package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Turns the robot to face our tag, LimeLightCamera does the seeing.
// Two ways to use it:
//   new AutoAlign(camera)              -> only works out the turn, call turnPower() and drive it yourself (Teleop, through Pedro)
//   new AutoAlign(hardwareMap, camera) -> also owns the wheels, call align() and it turns them (ShooterTuner)
public class AutoAlign {

    private DcMotor frwheel;   // null when built without a hardwareMap
    private DcMotor flwheel;
    private DcMotor blwheel;
    private DcMotor brwheel;
    private LimeLightCamera camera;

    // Limelight 3A sees about +/-27 deg horizontally, so 0.5 / 25 deg ~= 0.02
    private static final double kP = 0.02;
    private static final double MAX_TURN = 0.5;   // fastest it's allowed to turn
    private static final double MIN_TURN = 0.075;  // smallest push that still moves the robot
    private static final double TOLERANCE = 1.0;  // degrees of tx considered aligned

    // turn math only, something else drives the wheels
    public AutoAlign(LimeLightCamera camera) {
        this.camera = camera;
    }

    // turn math plus the drive motors, align() sets their power directly
    public AutoAlign(HardwareMap hardwareMap, LimeLightCamera camera) {
        this(camera);

        frwheel = hardwareMap.get(DcMotor.class, "fr");
        flwheel = hardwareMap.get(DcMotor.class, "fl");
        brwheel = hardwareMap.get(DcMotor.class, "br");
        blwheel = hardwareMap.get(DcMotor.class, "bl");
        flwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        blwheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // true when our tag is in view and within TOLERANCE of the crosshair
    public boolean isAligned() {
        double tx = camera.getTargetTx();
        return !Double.isNaN(tx) && Math.abs(tx) < TOLERANCE;
    }

    // Turn power toward the tag, positive turns right (same as rx in Teleop).
    // 0 when aligned or when the tag isn't visible.
    public double turnPower() {
        double tx = camera.getTargetTx();
        if (Double.isNaN(tx) || Math.abs(tx) < TOLERANCE) {
            return 0;
        }

        double power = kP * tx;
        if (Math.abs(power) < MIN_TURN) {
            power = Math.copySign(MIN_TURN, power);
        }
        return Math.max(-MAX_TURN, Math.min(MAX_TURN, power));
    }

    // Positive power turns right, matching rx in Teleop
    private void turn(double power) {
        if (flwheel == null) {
            throw new IllegalStateException("AutoAlign was built without a hardwareMap, use turnPower() instead");
        }
        flwheel.setPower(power);
        blwheel.setPower(power);
        frwheel.setPower(-power);
        brwheel.setPower(-power);
    }

    public void stop() {
        turn(0);
    }

    // Call once per loop. Turns the wheels itself. Returns true when facing the tag.
    public boolean align() {
        turn(turnPower());
        return isAligned();
    }
}

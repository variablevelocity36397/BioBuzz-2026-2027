package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsytems.TurretTrack;

/**
 * Finds the TurretTrack direction constants. Both servos and the encoder are FORWARD here.
 * Hold left bumper / trigger = turretleft at +0.5 / -0.5, right bumper / trigger = turretright at +0.5 / -0.5.
 * A (Cross) re-zeroes the encoder. Turret is zeroed at play (point it forward).
 */
@TeleOp(name = "Test Turret Directions", group = "Test")
public class TestTurretDirections extends OpMode {
    private static final double POWER = 0.5;
    private static final int STILL_TICKS = 5; // per loop, below this the turret counts as not moving

    private CRServoImplEx left, right;
    private DcMotor encoder; // brwheel drive motor: only its encoder is read, never set power
    private int lastTicks;
    private String ticksDir = "still";
    // +1 = +power makes ticks go up, -1 = down, 0 = not seen yet. Learned from the bumpers / triggers.
    private int leftSign, rightSign;

    @Override
    public void init() {
        left = hardwareMap.get(CRServoImplEx.class, "turretleft");
        right = hardwareMap.get(CRServoImplEx.class, "turretright");
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        encoder = hardwareMap.get(DcMotor.class, "brwheel");
        unpowerAll(); // limp during init: turn the turret by hand
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Point the turret straight forward by hand, then press play.");
        telemetry.update();
    }

    @Override
    public void start() {
        unpowerAll();
        zero();
    }

    @Override
    public void stop() {
        unpowerAll();
    }

    /** No signal at all = servo limp. A 0-power signal still makes it hold / fight the other servo. */
    private void unpowerAll() {
        left.setPwmDisable();
        right.setPwmDisable();
    }

    /** Only place a servo gets powered: called only while its button is held. */
    private static void power(CRServoImplEx servo, double p) {
        servo.setPower(p);
        servo.setPwmEnable();
    }

    private void zero() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastTicks = 0;
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) zero();
        int ticks = encoder.getCurrentPosition();
        int delta = ticks - lastTicks;
        lastTicks = ticks;
        int moving = delta > STILL_TICKS ? 1 : delta < -STILL_TICKS ? -1 : 0;
        ticksDir = moving > 0 ? "UP" : moving < 0 ? "DOWN" : "still";

        double lp = gamepad1.left_bumper ? POWER : gamepad1.left_trigger > 0.5 ? -POWER : 0;
        double rp = gamepad1.right_bumper ? POWER : gamepad1.right_trigger > 0.5 ? -POWER : 0;

        // Safety: TurretTrack soft limits. Encoder sign isn't known yet, so use the tighter limit on both sides.
        // Past it, a servo may only run in the direction already seen to move the ticks back toward 0.
        double angle = ticks / TurretTrack.TICKS_PER_DEGREE;
        boolean pastLimit = Math.abs(angle) >= Math.min(TurretTrack.MAX_DEGREE, -TurretTrack.MIN_DEGREE);
        if (pastLimit) {
            if (Math.signum(lp) * leftSign != -Math.signum(angle)) lp = 0;
            if (Math.signum(rp) * rightSign != -Math.signum(angle)) rp = 0;
        }
        // Both together (overrides bumpers/triggers): + = turret left, using TurretTrack's direction constants
        double turretAngle = (TurretTrack.ENCODER_REVERSED ? -1 : 1) * angle;
        boolean both = gamepad1.dpad_left || gamepad1.dpad_right;
        if (both) {
            double t = gamepad1.dpad_left ? POWER : -POWER;
            // Safety: real directional soft limits, never drive further into one
            if ((turretAngle >= TurretTrack.MAX_DEGREE && t > 0) || (turretAngle <= TurretTrack.MIN_DEGREE && t < 0)) t = 0;
            lp = (TurretTrack.LEFT_REVERSED ? -1 : 1) * t;
            rp = (TurretTrack.RIGHT_REVERSED ? -1 : 1) * t;
        }
        // Every loop: a servo whose button isn't held is unpowered; a held one keeps a steady signal (no off/on blink)
        if (lp != 0) power(left, lp); else left.setPwmDisable();
        if (rp != 0) power(right, rp); else right.setPwmDisable();
        // Only learn from a servo running alone, otherwise the tick change can't be blamed on one of them
        if (lp != 0 && rp == 0 && moving != 0) leftSign = (int) Math.signum(lp) * moving;
        if (rp != 0 && lp == 0 && moving != 0) rightSign = (int) Math.signum(rp) * moving;

        if (pastLimit) telemetry.addLine("!!! PAST SOFT LIMIT - only the direction back toward forward runs (or turn it by hand) !!!");
        telemetry.addLine("1. Turn the turret LEFT (CCW from above) by hand:");
        telemetry.addLine("   ticks UP -> ENCODER_REVERSED = false, DOWN -> true");
        telemetry.addLine("2. Hold each bumper (+0.5): if the turret turns LEFT, that servo's REVERSED = false, RIGHT -> true");
        telemetry.addLine("3. Dpad Left / Right = both servos turn turret LEFT / RIGHT (should be smooth, no grinding)");
        telemetry.addData("Dpad - both servos", !both ? "off" : lp == 0 ? "BLOCKED at soft limit" : gamepad1.dpad_left ? "LEFT" : "RIGHT");
        telemetry.addData("Turret angle (deg, + = left)", "%.1f", turretAngle);
        telemetry.addData("A (Cross) - re-zero encoder", "point turret forward first");
        telemetry.addData("Encoder ticks", ticks);
        telemetry.addData("Ticks moving", ticksDir);
        telemetry.addData("Angle if not reversed (deg)", "%.1f", angle);
        telemetry.addData("turretleft (LB +0.5 / LT -0.5)", "%.1f", lp);
        telemetry.addData("turretright (RB +0.5 / RT -0.5)", "%.1f", rp);
        telemetry.addData("turretleft +0.5 makes", result(leftSign));
        telemetry.addData("turretright +0.5 makes", result(rightSign));
        telemetry.update();
    }

    private static String result(int sign) {
        return sign > 0 ? "ticks UP" : sign < 0 ? "ticks DOWN" : "not run yet";
    }
}

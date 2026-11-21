package org.firstinspires.ftc.teamcode.subsytems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Turret on 2 continuous servos, aimed at the nearest alliance goal from the odometry pose.
 * Angle comes from a Through Bore encoder on the brwheel port. 0 deg = forward, + = left (CCW from above).
 * Field frame = Pedro (inches, heading CCW from +X, radians in Pose).
 */
public class TurretTrack {
    public enum Alliance {
        // start pose (x, y, heading deg), goal on the 3-nectar side, other goal
        BLUE(new Pose(133, 133.3, Math.toRadians(270)), new double[][]{{84, 85}, {84, 56}}),
        RED(new Pose(8.3, 8.4, Math.toRadians(90)), new double[][]{{58, 85}, {58, 56}});

        public final Pose start;
        final double[][] goals;

        Alliance(Pose start, double[][] goals) {
            this.start = start;
            this.goals = goals;
        }
    }

    // CONFIRM: servos face each other, so one is reversed. + power must turn the turret left.
    public static final boolean LEFT_REVERSED = true;  // L2 (-0.5) turned turret left
    public static final boolean RIGHT_REVERSED = true; // R1 (+0.5) turned turret right
    // CONFIRM: turn the turret left by hand, the angle must go up
    public static final boolean ENCODER_REVERSED = false;
    // Through Bore 8192 ticks/rev, encoder turns 2x per turret turn (encoder 1 : turret 2)
    public static final double ENCODER_TICKS_PER_REV = 8192;
    public static final double GEAR_RATIO = 2;
    public static final double TICKS_PER_DEGREE = ENCODER_TICKS_PER_REV * GEAR_RATIO / 360;
    // Soft limits: protect the turret wiring (no slip ring). Kept at +-90 until the -210 / 150 range is verified.
    public static final double MIN_DEGREE = -90;
    public static final double MAX_DEGREE = 90;
    public static final double TURRET_ZERO_OFFSET = 0;  // deg, if "forward" at zeroing isn't truly forward
    public static final double TURRET_OFFSET_X = 0;     // in, pivot forward of the tracking center
    public static final double TURRET_OFFSET_Y = 0;     // in, pivot left of the tracking center
    public static final double KP = 0.007; // tuned on robot
    public static final double KD = 0.00; // start value, tune in Test Turret Track
    public static final double MAX_POWER = 0.6;
    public static final double MIN_POWER = 0.072;       // overcomes static friction, tuned on robot
    public static final double DEADBAND_DEG = 1;
    public static final double GOAL_SWITCH_HYSTERESIS = 3; // in, other goal must be this much closer to switch
    // Aim x shifts with the shot's sideways lean: red corner (8.3, 8.4) -> goal (58, 56) aims at x 65.5, straight on aims at center
    public static final double AIM_OFFSET_AT_CORNER = 7.5; // in, also the max shift
    public static final double CORNER_LEAN_RAD = Math.atan2(58 - 8.3, 56 - 8.4);

    private final CRServoImplEx left, right;
    private final DcMotor encoder; // brwheel drive motor: only its encoder is read here, never set power
    private final ElapsedTime dt = new ElapsedTime();
    private Alliance alliance = Alliance.BLUE;
    private int goal; // index into alliance.goals
    private double kp = KP, kd = KD;
    private double angle, target, error, lastError, power, fieldAngleDeg, aimX;
    private boolean atLimit;

    public TurretTrack(HardwareMap hw) {
        left = hw.get(CRServoImplEx.class, "turretleft");
        right = hw.get(CRServoImplEx.class, "turretright");
        left.setDirection(LEFT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(RIGHT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        encoder = hw.get(DcMotor.class, "brwheel");
        // No signal until zero(): a 0-power pulse can still move the servos during init
        left.setPwmDisable();
        right.setPwmDisable();
    }

    /** Call at start() with the turret pointed straight forward by hand: 0 ticks = 0 deg. */
    public void zero() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastError = 0;
        dt.reset();
        setPower(0);
        left.setPwmEnable();
        right.setPwmEnable();
    }

    public void setAlliance(Alliance a) {
        alliance = a;
        goal = 0;
    }

    /** track = aim at the nearest goal, otherwise hold forward. Call every loop. */
    public void update(Pose robot, boolean track) {
        angle = getAngleDeg();
        double h = robot.heading();
        // Turret pivot in field frame
        double px = robot.x() + TURRET_OFFSET_X * Math.cos(h) - TURRET_OFFSET_Y * Math.sin(h);
        double py = robot.y() + TURRET_OFFSET_X * Math.sin(h) + TURRET_OFFSET_Y * Math.cos(h);

        double[][] g = alliance.goals;
        int other = 1 - goal;
        if (Math.hypot(g[other][0] - px, g[other][1] - py) < Math.hypot(g[goal][0] - px, g[goal][1] - py) - GOAL_SWITCH_HYSTERESIS) {
            goal = other;
        }
        // Signed lean (+ = goal to +x of the turret) mirrors the shift for both sides and both alliances
        double lean = Math.atan2(g[goal][0] - px, Math.abs(g[goal][1] - py));
        aimX = g[goal][0] + Math.max(-AIM_OFFSET_AT_CORNER, Math.min(AIM_OFFSET_AT_CORNER, AIM_OFFSET_AT_CORNER * lean / CORNER_LEAN_RAD));
        double fieldAngle = Math.atan2(g[goal][1] - py, aimX - px);
        fieldAngleDeg = Math.toDegrees(fieldAngle);
        target = track ? reachable(normalize(Math.toDegrees(fieldAngle - h) - TURRET_ZERO_OFFSET)) : 0;

        error = target - angle;
        double d = dt.seconds() > 0 ? (error - lastError) / dt.seconds() : 0;
        lastError = error;
        dt.reset();
        double p = 0;
        if (Math.abs(error) > DEADBAND_DEG) {
            p = Math.max(-MAX_POWER, Math.min(MAX_POWER, kp * error + kd * d));
            if (Math.abs(p) < MIN_POWER) p = Math.signum(p) * MIN_POWER;
        }
        // Safety: never drive further into a soft limit
        atLimit = angle >= MAX_DEGREE || angle <= MIN_DEGREE;
        if ((angle >= MAX_DEGREE && p > 0) || (angle <= MIN_DEGREE && p < 0)) p = 0;
        setPower(p);
    }

    /** Equivalent of deg (+-360) inside the limits closest to the current angle, else the nearest limit. */
    private double reachable(double deg) {
        double best = Double.NaN;
        for (double c : new double[]{deg, deg + 360, deg - 360}) {
            if (c >= MIN_DEGREE && c <= MAX_DEGREE && (Double.isNaN(best) || Math.abs(c - angle) < Math.abs(best - angle))) best = c;
        }
        if (!Double.isNaN(best)) return best;
        return Math.abs(normalize(deg - MIN_DEGREE)) < Math.abs(normalize(deg - MAX_DEGREE)) ? MIN_DEGREE : MAX_DEGREE;
    }

    private static double normalize(double deg) {
        deg %= 360;
        if (deg > 180) deg -= 360;
        if (deg < -180) deg += 360;
        return deg;
    }

    private void setPower(double p) {
        power = p;
        left.setPower(p);
        right.setPower(p);
    }

    public double getAngleDeg() {
        return (ENCODER_REVERSED ? -1 : 1) * encoder.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public Alliance getAlliance() { return alliance; }
    public double getKp() { return kp; }
    public void setKp(double kp) { this.kp = Math.max(0, kp); }
    public double getKd() { return kd; }
    public void setKd(double kd) { this.kd = Math.max(0, kd); }
    public double getTarget() { return target; }
    public double getError() { return error; }
    public double getPower() { return power; }
    public double getFieldAngleDeg() { return fieldAngleDeg; }
    public double getGoalX() { return alliance.goals[goal][0]; }
    public double getGoalY() { return alliance.goals[goal][1]; }
    public double getAimX() { return aimX; }
    public boolean isAtLimit() { return atLimit; }
}

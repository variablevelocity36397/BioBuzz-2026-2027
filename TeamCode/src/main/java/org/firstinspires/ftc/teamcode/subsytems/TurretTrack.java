package org.firstinspires.ftc.teamcode.subsytems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Turret on 2 continuous servos (wooden robot), aimed at the nearest alliance goal from the odometry pose.
 * A motor-driven turret (metal robot) can reuse all of this through the protected constructor; only hardware and
 * tuned values differ.
 * Angle comes from a Through Bore encoder on the brwheel port. 0 deg = forward, + = left (CCW from above).
 * Field frame = Pedro (inches, heading CCW from +X, radians in Pose).
 */
public class TurretTrack {
    public enum Alliance {
        // start pose (x, y, heading deg), {goal on the 3-nectar side, other goal}
        BLUE(new Pose(133.6, 132, Math.toRadians(270)), new double[][]{{84.1, 90}, {84.1, 51.75}}),
        RED(new Pose(10.4, 12, Math.toRadians(90)), new double[][]{{58, 51.75}, {58, 90}});

        public final Pose start;
        final double[][] goals;

        Alliance(Pose start, double[][] goals) {
            this.start = start;
            this.goals = goals;
        }
    }

    // Measured on robot 2026-11-25 (red, heading 90, turret moved by hand): turret deg each shot needs, rows by AIM_YS.
    // (34.4, 36) and (60, 36) can't be reached at shooter velocity 1100. Blue uses the point mirror (144 - x, 144 - y).
    public static final double[] AIM_XS = {10.4, 34.4, 60};
    public static final double[] AIM_YS = {12, 36};
    public static final double[][] AIM_TURRET_DEG = {{-53.3, -28.2, 0}, {-73, -48.8, 0}};
    public static final double AIM_HEADING_DEG = 90;
    public static final double[] AIM_GOAL = Alliance.RED.goals[0]; // what the points were measured against
    // Far goal (goals[1]) has no measured grid: the near grid is mirrored top to bottom about the midpoint between the
    // goals (y = 12 -> 129.75, 36 -> 105.75), heading 90 -> 270, and the turn flipped (-1). Untested: if far shots
    // miss by about the same amount on the other side, set +1.
    public static final double FAR_GOAL_Y = Alliance.RED.goals[1][1];
    public static final double FAR_GOAL_SIGN = -1;
    // Measured minus straight at the goal center, deg
    private static final double[][] AIM_CORR = new double[AIM_YS.length][AIM_XS.length];
    static {
        for (int j = 0; j < AIM_YS.length; j++) for (int i = 0; i < AIM_XS.length; i++) {
            double straight = Math.toDegrees(Math.atan2(AIM_GOAL[1] - AIM_YS[j], AIM_GOAL[0] - AIM_XS[i]));
            AIM_CORR[j][i] = normalize(AIM_TURRET_DEG[j][i] + AIM_HEADING_DEG - straight);
        }
    }

    // CONFIRM: servos face each other, so one is reversed. + power must turn the turret left.
    public static final boolean LEFT_REVERSED = true;  // L2 (-0.5) turned turret left
    public static final boolean RIGHT_REVERSED = true; // R1 (+0.5) turned turret right
    // Confirmed on robot: turning the turret left raises the angle
    public static final boolean ENCODER_REVERSED = false;
    // Through Bore 8192 ticks/rev, encoder turns 2x per turret turn (encoder 1 : turret 2)
    public static final double ENCODER_TICKS_PER_REV = 8192;
    public static final double GEAR_RATIO = 2;
    public static final double TICKS_PER_DEGREE = ENCODER_TICKS_PER_REV * GEAR_RATIO / 360;
    // Soft limits: protect the turret wiring (no slip ring). Set by the team: 205 deg right, 130 deg left (confirmed 2026-11-25).
    public static final double MIN_DEGREE = -205; // right
    public static final double MAX_DEGREE = 130;  // left
    public static final double TURRET_ZERO_OFFSET = 0;  // deg, if "forward" at zeroing isn't truly forward
    public static final double TURRET_OFFSET_X = 0;     // in, pivot forward of the tracking center
    public static final double TURRET_OFFSET_Y = 0;     // in, pivot left of the tracking center
    public static final double KP = 0.007; // tuned on robot
    public static final double KD = 0.0006; // tuned in Turret Tuner at MAX_POWER 1
    public static final double MAX_POWER = 1;
    public static final double MIN_POWER = 0.075;       // overcomes static friction (breaks away at 0.068-0.072), tuned on robot
    public static final double DEADBAND_DEG = 1;
    // Safety: a bigger change than this in one loop is not real motion (encoder reset or glitch), so the turret stops
    public static final double ENCODER_JUMP_DEG = 30;
    public static final double GOAL_SWITCH_HYSTERESIS = 3; // in, other goal must be this much closer to switch

    private final DcMotorSimple[] outputs; // all get the same power (+ = left)
    private final DcMotor encoder; // wooden: brwheel drive motor, only its encoder is read here, never set power
    private final boolean encoderReversed;
    private final double ticksPerDegree, minDegree, maxDegree; // per robot, minDegree/maxDegree = soft limits
    private final ElapsedTime dt = new ElapsedTime();
    private Alliance alliance = Alliance.BLUE;
    private int goal; // index into alliance.goals
    private final double minPower, maxPower;
    private double kp, kd, deadband = DEADBAND_DEG;
    private double angle, target, error, lastError, power, fieldAngleDeg;
    private boolean atLimit, encoderFault, hasAngle;

    /** Wooden robot: 2 continuous servos. */
    public TurretTrack(HardwareMap hw) {
        this(hw.get(DcMotor.class, "brwheel"), ENCODER_REVERSED, TICKS_PER_DEGREE, MIN_DEGREE, MAX_DEGREE,
                KP, KD, MIN_POWER, MAX_POWER, servo(hw, "turretleft", LEFT_REVERSED), servo(hw, "turretright", RIGHT_REVERSED));
    }

    /** Any turret drive: outputs get the same power, + must turn the turret left. Limits in deg (+ = left). */
    protected TurretTrack(DcMotor encoder, boolean encoderReversed, double ticksPerDegree, double minDegree,
                          double maxDegree, double kp, double kd, double minPower, double maxPower,
                          DcMotorSimple... outputs) {
        this.encoder = encoder;
        this.encoderReversed = encoderReversed;
        this.ticksPerDegree = ticksPerDegree;
        this.minDegree = minDegree;
        this.maxDegree = maxDegree;
        this.kp = kp;
        this.kd = kd;
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.outputs = outputs;
    }

    private static CRServoImplEx servo(HardwareMap hw, String name, boolean reversed) {
        CRServoImplEx s = hw.get(CRServoImplEx.class, name);
        s.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        s.setPwmDisable(); // No signal until zero(): a 0-power pulse can still move the servos during init
        return s;
    }

    /** Call at start() with the turret pointed straight forward by hand: 0 ticks = 0 deg. */
    public void zero() {
        resetEncoder();
        lastError = 0;
        dt.reset();
        setPower(0);
        for (DcMotorSimple o : outputs) if (o instanceof CRServoImplEx) ((CRServoImplEx) o).setPwmEnable();
    }

    /** Current turret position = 0 deg. Servos stay as they are (unpowered in init). */
    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hasAngle = false; // the jump to 0 here is expected, not a fault
    }

    public void setAlliance(Alliance a) {
        alliance = a;
        goal = 0;
    }

    /** track = aim at the nearest goal, otherwise hold forward. Call every loop. */
    public void update(Pose robot, boolean track) {
        readAngle();
        double aim = aimAngle(robot);
        target = track ? reachable(aim) : 0;
        runPd();
    }

    /**
     * Turret angle (deg, -180..180, before soft limits) that aims at the nearest goal from this pose: goal center plus
     * the measured correction. Never moves the turret.
     */
    public double aimAngle(Pose robot) {
        double[] p = pivot(robot);
        double[] g = alliance.goals[pickGoal(robot)];
        fieldAngleDeg = Math.toDegrees(Math.atan2(g[1] - p[1], g[0] - p[0])) + aimCorrection(robot);
        return normalize(fieldAngleDeg - Math.toDegrees(robot.heading()) - TURRET_ZERO_OFFSET);
    }

    /** Index into the alliance goals (0 = 3-nectar goal) of the nearer goal, with hysteresis so it doesn't flicker. */
    public int pickGoal(Pose robot) {
        double[] p = pivot(robot);
        double[][] g = alliance.goals;
        int other = 1 - goal;
        if (Math.hypot(g[other][0] - p[0], g[other][1] - p[1]) < Math.hypot(g[goal][0] - p[0], g[goal][1] - p[1]) - GOAL_SWITCH_HYSTERESIS) {
            goal = other;
        }
        return goal;
    }

    /**
     * Deg to add to the straight-at-goal aim, bilinear in the measured grid (ends held), for the goal from the last
     * pickGoal(). goals[1] reuses the grid mirrored (see FAR_GOAL_SIGN). The 180 deg point mirror keeps CCW angles, so blue uses the same sign.
     */
    public double aimCorrection(Pose robot) {
        double[] p = pivot(robot);
        boolean blue = alliance == Alliance.BLUE;
        double x = blue ? 144 - p[0] : p[0], y = blue ? 144 - p[1] : p[1];
        if (goal != 0) return FAR_GOAL_SIGN * gridCorrection(x, AIM_GOAL[1] + FAR_GOAL_Y - y); // mirror into the measured grid
        return gridCorrection(x, y);
    }

    private static double gridCorrection(double x, double y) {
        int i = cell(AIM_XS, x), j = cell(AIM_YS, y);
        double tx = frac(AIM_XS, i, x), ty = frac(AIM_YS, j, y);
        double lo = AIM_CORR[j][i] + tx * (AIM_CORR[j][i + 1] - AIM_CORR[j][i]);
        double hi = AIM_CORR[j + 1][i] + tx * (AIM_CORR[j + 1][i + 1] - AIM_CORR[j + 1][i]);
        return lo + ty * (hi - lo);
    }

    // Lower index of the grid cell holding v (first / last cell past the ends)
    private static int cell(double[] a, double v) {
        int i = 0;
        while (i < a.length - 2 && v > a[i + 1]) i++;
        return i;
    }

    // Position of v in cell i, 0..1 (clamped, so ends are held)
    private static double frac(double[] a, int i, double v) {
        return Math.max(0, Math.min(1, (v - a[i]) / (a[i + 1] - a[i])));
    }

    /** Turret pivot in field frame (in). */
    public static double[] pivot(Pose robot) {
        double h = robot.heading();
        return new double[]{robot.x() + TURRET_OFFSET_X * Math.cos(h) - TURRET_OFFSET_Y * Math.sin(h),
                robot.y() + TURRET_OFFSET_X * Math.sin(h) + TURRET_OFFSET_Y * Math.cos(h)};
    }

    /** Distance (in) from the turret pivot to the nearer of our alliance's goals. */
    public double goalDistance(Pose robot) {
        double[] p = pivot(robot);
        double d = Double.MAX_VALUE;
        for (double[] g : alliance.goals) d = Math.min(d, Math.hypot(g[0] - p[0], g[1] - p[1]));
        return d;
    }

    /** PD to a turret angle (deg, + = left), clamped to the soft limits. Call every loop. */
    public void aimAt(double deg) {
        readAngle();
        target = Math.max(minDegree, Math.min(maxDegree, deg));
        runPd();
    }

    /** Open-loop power (+ = left), soft limits applied. Call every loop. */
    public void manual(double p) {
        readAngle();
        setLimitedPower(p);
    }

    private void runPd() {
        error = target - angle;
        double d = dt.seconds() > 0 ? (error - lastError) / dt.seconds() : 0;
        lastError = error;
        dt.reset();
        double p = 0;
        if (Math.abs(error) > deadband) {
            p = Math.max(-maxPower, Math.min(maxPower, kp * error + kd * d));
            if (Math.abs(p) < minPower) p = Math.signum(p) * minPower;
        }
        setLimitedPower(p);
    }

    // Safety: an impossible jump means the encoder was reset or glitched, so the soft limits can't be trusted.
    // Latched until the OpMode restarts (re-zero by hand).
    private void readAngle() {
        double a = getAngleDeg();
        if (hasAngle && Math.abs(a - angle) > ENCODER_JUMP_DEG) encoderFault = true;
        angle = a;
        hasAngle = true;
    }

    // Safety: never drive further into a soft limit, and never move with an encoder fault
    private void setLimitedPower(double p) {
        if (encoderFault) p = 0;
        atLimit = angle >= maxDegree || angle <= minDegree;
        if ((angle >= maxDegree && p > 0) || (angle <= minDegree && p < 0)) p = 0;
        setPower(p);
    }

    /** Equivalent of deg (+-360) inside the limits closest to the current angle, else the nearest limit. */
    private double reachable(double deg) {
        double best = Double.NaN;
        for (double c : new double[]{deg, deg + 360, deg - 360}) {
            if (c >= minDegree && c <= maxDegree && (Double.isNaN(best) || Math.abs(c - angle) < Math.abs(best - angle))) best = c;
        }
        if (!Double.isNaN(best)) return best;
        return Math.abs(normalize(deg - minDegree)) < Math.abs(normalize(deg - maxDegree)) ? minDegree : maxDegree;
    }

    public static double normalize(double deg) {
        deg %= 360;
        if (deg > 180) deg -= 360;
        if (deg < -180) deg += 360;
        return deg;
    }

    private void setPower(double p) {
        power = p;
        for (DcMotorSimple o : outputs) o.setPower(p);
    }

    public double getAngleDeg() {
        return (encoderReversed ? -1 : 1) * encoder.getCurrentPosition() / ticksPerDegree;
    }

    public Alliance getAlliance() { return alliance; }
    public double getKp() { return kp; }
    public void setKp(double kp) { this.kp = Math.max(0, kp); }
    public double getKd() { return kd; }
    public void setKd(double kd) { this.kd = Math.max(0, kd); }
    public double getDeadband() { return deadband; }
    public void setDeadband(double deg) { deadband = Math.max(0, deg); }
    public double getTarget() { return target; }
    public double getError() { return error; }
    public double getPower() { return power; }
    public double getFieldAngleDeg() { return fieldAngleDeg; }
    public double getGoalX() { return alliance.goals[goal][0]; }
    public double getGoalY() { return alliance.goals[goal][1]; }
    public boolean isAtLimit() { return atLimit; }
    public boolean isEncoderFault() { return encoderFault; }
}

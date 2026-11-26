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
        // start pose (x, y, heading deg), {goal on the 3-nectar side, other goal}, aim table rows at goals[0]
        // Rows: {pivot x, pivot y, turret field angle deg, shooter tx deg, ty deg}, pasted from Aim Table Tuner.
        // placeholder: replace with Aim Table Tuner rows (straight on + corner, from the old cornerAimX 75 / 67.5; ty unknown)
        BLUE(new Pose(133.6, 132, Math.toRadians(270)), new double[][]{{84.1, 90}, {84.1, 51.75}}, new double[][]{
                {84.1, 133.3, -90, 0, 0},
                {133, 133.3, -143.26, 4.78, 0}}),
        RED(new Pose(10.4, 12, Math.toRadians(90)), new double[][]{{58, 51.75}, {58, 90}}, new double[][]{
                {58, 8.4, 90, 0, 0},
                {8.3, 8.4, 36.21, 4.88, 0}});

        public final Pose start;
        final double[][] goals;
        // Sorted by lean: {|lean| rad, aim correction deg, shooter tx deg, ty deg}, mirrored to lean >= 0, goal above
        final double[][] table;

        Alliance(Pose start, double[][] goals, double[][] rows) {
            this.start = start;
            this.goals = goals;
            double[] g = goals[0];
            table = new double[rows.length][];
            for (int i = 0; i < rows.length; i++) {
                double[] r = rows[i];
                double lean = lean(g, r[0], r[1]), m = mirror(lean, g[1] - r[1]);
                double corr = normalize(r[2] - Math.toDegrees(Math.atan2(g[1] - r[1], g[0] - r[0])));
                table[i] = new double[]{Math.abs(lean), m * corr, m * r[3], r[4]};
            }
            java.util.Arrays.sort(table, (a, b) -> Double.compare(a[0], b[0]));
        }
    }

    // Measured on robot 2026-11-25 (red, heading 90, turret moved by hand): turret deg each shot needs, rows by AIM_YS.
    // (34.4, 36) and (60, 36) can't be reached at shooter velocity 1100. Blue uses the point mirror (144 - x, 144 - y).
    public static final double[] AIM_XS = {10.4, 34.4, 60};
    public static final double[] AIM_YS = {12, 36};
    public static final double[][] AIM_TURRET_DEG = {{-53.3, -28.2, 0}, {-73, -48.8, 0}};
    public static final double AIM_HEADING_DEG = 90;
    public static final double[] AIM_GOAL = Alliance.RED.goals[0]; // what the points were measured against
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

    private final CRServoImplEx left, right;
    private final DcMotor encoder; // brwheel drive motor: only its encoder is read here, never set power
    private final ElapsedTime dt = new ElapsedTime();
    private Alliance alliance = Alliance.BLUE;
    private int goal; // index into alliance.goals
    private double kp = KP, kd = KD, minPower = MIN_POWER, maxPower = MAX_POWER, deadband = DEADBAND_DEG;
    private boolean softLimits = true; // only TurretTuner turns these off (wiring removed for tuning)
    private double angle, target, error, lastError, power, fieldAngleDeg;
    private boolean atLimit, encoderFault, hasAngle;

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
        resetEncoder();
        lastError = 0;
        dt.reset();
        setPower(0);
        left.setPwmEnable();
        right.setPwmEnable();
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
     * the measured correction. Aim table on hold, not used for aiming. Never moves the turret.
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
     * Aim table interpolated at this pose for the goal from the last pickGoal():
     * {aim correction deg (added to the angle at the goal center), shooter tx deg (tag tx the camera should hold,
     * camera offset removed), expected ty deg}. Linear in |lean| between rows, ends held past the table.
     */
    public double[] aimTable(Pose robot) {
        double[] p = pivot(robot), g = alliance.goals[goal];
        double lean = lean(g, p[0], p[1]), m = mirror(lean, g[1] - p[1]), a = Math.abs(lean);
        double[][] r = alliance.table;
        int i = 0;
        while (i < r.length - 2 && a > r[i + 1][0]) i++;
        double[] lo = r[i], hi = r[Math.min(i + 1, r.length - 1)];
        double span = hi[0] - lo[0];
        double t = span > 0 ? Math.max(0, Math.min(1, (a - lo[0]) / span)) : 0;
        return new double[]{m * (lo[1] + t * (hi[1] - lo[1])), m * (lo[2] + t * (hi[2] - lo[2])), lo[3] + t * (hi[3] - lo[3])};
    }

    /**
     * Deg to add to the straight-at-goal aim, bilinear in the measured grid (ends held), for the goal from the last
     * pickGoal(). 0 for goals[1] (not measured). The 180 deg point mirror keeps CCW angles, so blue uses the same sign.
     */
    public double aimCorrection(Pose robot) {
        if (goal != 0) return 0;
        double[] p = pivot(robot);
        boolean blue = alliance == Alliance.BLUE;
        double x = blue ? 144 - p[0] : p[0], y = blue ? 144 - p[1] : p[1];
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

    /** Signed lean (rad) from (x, y) to goal g: + = goal to +x. 0 = straight on. */
    static double lean(double[] g, double x, double y) {
        return Math.atan2(g[0] - x, Math.abs(g[1] - y));
    }

    // Flips aim angles between the 4 mirrored spots (either side of the goal, goal above or below), like the old aim-x shift
    static double mirror(double lean, double dy) {
        return (lean >= 0 ? 1 : -1) * (dy >= 0 ? 1 : -1);
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

    /** PD to a turret angle (deg, + = left), clamped to the soft limits unless disabled. Call every loop. */
    public void aimAt(double deg) {
        readAngle();
        target = softLimits ? Math.max(MIN_DEGREE, Math.min(MAX_DEGREE, deg)) : deg;
        runPd();
    }

    /** Open-loop power (+ = left), soft limits applied unless disabled. Call every loop. */
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
        atLimit = angle >= MAX_DEGREE || angle <= MIN_DEGREE;
        if (softLimits && ((angle >= MAX_DEGREE && p > 0) || (angle <= MIN_DEGREE && p < 0))) p = 0;
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

    public static double normalize(double deg) {
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
    public void setMinPower(double p) { minPower = Math.max(0, p); }
    public double getDeadband() { return deadband; }
    public void setDeadband(double deg) { deadband = Math.max(0, deg); }
    public void setMaxPower(double p) { maxPower = Math.max(0, Math.min(1, p)); }
    /** Tuning only, with the turret wiring removed. Never call from TeleOp or autos. */
    public void disableSoftLimits() { softLimits = false; }
    public double getTarget() { return target; }
    public double getError() { return error; }
    public double getPower() { return power; }
    public double getFieldAngleDeg() { return fieldAngleDeg; }
    public double getGoalX() { return alliance.goals[goal][0]; }
    public double getGoalY() { return alliance.goals[goal][1]; }
    public boolean isAtLimit() { return atLimit; }
    public boolean isEncoderFault() { return encoderFault; }
}

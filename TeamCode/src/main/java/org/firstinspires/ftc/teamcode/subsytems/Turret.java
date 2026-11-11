package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Turret on 2 goBILDA 5-turn servos, aimed with the Limelight. 0 deg = forward, + = left (CCW from above). */
public class Turret {
    // Set these from what Test Directions shows
    public static final boolean LEFT_REVERSED = false;
    public static final boolean RIGHT_REVERSED = false;

    public static final int PIPELINE = 5;
    // 5-turn servo = 1800 deg. Servo 60T -> 20T (x3), 20T shares an axle with 31T, 31T -> 62T turret (x0.5).
    // Turret turns 3 * 31/62 = 1.5x the servo -> 2700 turret deg across servo 0..1
    public static final double TURRET_RANGE_DEG = 1800 * (60.0 / 20.0) * (31.0 / 62.0);
    // Soft limits: protect the turret wiring. Forward-facing only for now.
    public static final double MIN_DEG = -90;
    public static final double MAX_DEG = 90;
    public static final double KP = 0.5; // fraction of tx corrected per loop, tune on robot
    public static final double AIM_SIGN = 1; // flip if the turret turns away from the target
    // BIOBUZZ hive tags: red cells 30-33, 34-37; blue cells 38-41, 42-45
    public static final int FIRST_TAG = 30;
    public static final int TAGS_PER_CELL = 4;

    private final Servo left, right;
    private final Limelight3A limelight;
    private double angleDeg;
    private double tx;
    private boolean red;
    private int tagCount;
    private double lastFrameTime = Double.NaN;

    public Turret(HardwareMap hw) {
        left = hw.get(Servo.class, "turretleft");
        right = hw.get(Servo.class, "turretright");
        applyDirections(left, right);
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    public static void applyDirections(Servo left, Servo right) {
        left.setDirection(LEFT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        right.setDirection(RIGHT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    /** Clamps to the soft limits, then converts turret degrees to servo position. Every servo write goes through this. */
    public static double servoPosition(double deg) {
        return 0.5 + clamp(deg) / TURRET_RANGE_DEG;
    }

    private static double clamp(double deg) {
        return Math.max(MIN_DEG, Math.min(MAX_DEG, deg));
    }

    public void setAngle(double deg) {
        angleDeg = clamp(deg);
        left.setPosition(servoPosition(angleDeg));
        right.setPosition(servoPosition(angleDeg));
    }

    public void center() {
        setAngle(0);
    }

    public void setRed(boolean red) { this.red = red; }

    /**
     * Nudges toward our alliance's hive cell (camera is on the turret, so tx is the remaining error). Holds if none seen.
     * Each BIOBUZZ cell has a 4-tag cluster around its opening (IDs above FIRST_TAG).
     * Aims at the average tx of a cluster's visible tags (~opening center); prefers the cluster with the most tags seen
     * (most face-on), then the larger total area (closer).
     */
    public void aim() {
        LLResult result = limelight.getLatestResult();
        // Loop runs faster than the camera: only correct once per new frame, or the same tx gets applied twice
        if (result != null && result.getTimestamp() == lastFrameTime) return;
        if (result != null) lastFrameTime = result.getTimestamp();
        int first = red ? FIRST_TAG : FIRST_TAG + 2 * TAGS_PER_CELL; // each alliance has 2 cells
        int[] count = new int[2];
        double[] sumTx = new double[2], area = new double[2];
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id < first || id >= first + 2 * TAGS_PER_CELL) continue;
                int c = (id - first) / TAGS_PER_CELL;
                count[c]++;
                sumTx[c] += tag.getTargetXDegrees();
                area[c] += tag.getTargetArea();
            }
        }
        int best = count[1] > count[0] || (count[1] == count[0] && area[1] > area[0]) ? 1 : 0;
        tagCount = count[best];
        if (!hasTarget()) {
            tx = 0; // no tags: don't move, and don't show a stale tx
            return;
        }
        tx = sumTx[best] / count[best];
        setAngle(angleDeg + AIM_SIGN * KP * tx);
    }

    public int getTagCount() { return tagCount; }

    public double getAngleDeg() { return angleDeg; }

    public boolean hasTarget() { return tagCount > 0; }

    public boolean isRed() { return red; }

    public double getTx() { return tx; }
}

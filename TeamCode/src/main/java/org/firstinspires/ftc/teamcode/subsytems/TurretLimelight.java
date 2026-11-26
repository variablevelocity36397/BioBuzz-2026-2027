package org.firstinspires.ftc.teamcode.subsytems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.HiveTagCamera;

/**
 * Turret aimed with the Limelight (mounted on the turret, so tx is the remaining error).
 * All motion goes through TurretTrack.aimAt, so its PD, soft limits and encoder-fault stop stay in charge.
 */
public class TurretLimelight {
    public enum Mode { OFF, MANUAL, CAMERA, GRACE, ODOMETRY }

    // Set on robot 2026-11-22: +1 turns the turret toward the tags on this camera mount
    public static final double AIM_SIGN = 1;
    public static final double DEADBAND_DEG = 1.5; // set 2026-11-25 (was 3), tune in Test Turret Limelight (odometry keeps TurretTrack's 1)
    public static final double MAX_FRAME_AGE_MS = 100; // older pictures are ignored. Tune: ~2x the normal latency
    // Each new frame moves the camera target this fraction of the way (1 = no smoothing). Stops the jump when a tag
    // enters or leaves view and the tag average shifts. Lower = smoother but slower to follow. Tune on robot.
    public static final double SMOOTHING = 0.25;
    public static final double GRACE_MS = 100; // keep the camera target this long after the tags are lost, then odometry
    // Camera sits this far right of the shooter (parallel). Shooter must point atan(offset / distance) right of where
    // the camera centers the tags; distance comes from odometry. Min distance stops the offset blowing up up close.
    public static final double CAMERA_RIGHT_OF_SHOOTER_IN = 2;
    public static final double MIN_OFFSET_DISTANCE_IN = 12;

    private static final int HISTORY = 256; // > 0.5 s of loops
    private final long[] histTime = new long[HISTORY];
    private final double[] histAngle = new double[HISTORY], histHeading = new double[HISTORY];
    private int histCount, histHead;

    private final TurretTrack turret;
    private final HiveTagCamera camera;
    private Mode mode = Mode.ODOMETRY;
    private double deadband = DEADBAND_DEG; // camera modes only; odometry uses TurretTrack.DEADBAND_DEG
    private double base, baseHeading; // base = turret target at capture, baseHeading = robot heading then (deg)
    private long lastGood = Long.MIN_VALUE;
    private boolean seeing; // latest frame had usable tags
    private double offsetDeg; // last shooter offset used

    public TurretLimelight(HardwareMap hw) {
        turret = new TurretTrack(hw);
        camera = new HiveTagCamera(hw);
    }

    /** Call at start() with the turret pointed straight forward by hand. */
    public void zero() {
        turret.zero();
        mode = Mode.ODOMETRY;
        lastGood = Long.MIN_VALUE;
        seeing = false;
        histCount = 0;
    }

    /** Current turret position = 0 deg (servos unpowered in init). */
    public void resetEncoder() { turret.resetEncoder(); }

    public void setAlliance(TurretTrack.Alliance a) { turret.setAlliance(a); }

    /**
     * track = aim at our hive cell with the Limelight, otherwise hold forward. Tags lost: keep the camera target for
     * GRACE_MS, then aim with odometry (TurretTrack) until tags come back. robot = Pinpoint pose. Call every loop.
     */
    public void update(Pose robot, boolean track) {
        long now = System.nanoTime();
        double angle = turret.getAngleDeg();
        double heading = Math.toDegrees(robot.heading());
        record(now, angle, heading);
        // Camera uses only the cell of the goal odometry picks (nearer, with hysteresis), so both modes aim at the same goal.
        // goals[0] (3-nectar) is the second tag group on both alliances (red 34-37, blue 42-45), so cell = 1 - goal.
        boolean newFrame = camera.update(turret.getAlliance() == TurretTrack.Alliance.RED, 1 - turret.pickGoal(robot));

        if (!track) {
            mode = Mode.OFF;
            lastGood = Long.MIN_VALUE;
            seeing = false;
            turret.setDeadband(TurretTrack.DEADBAND_DEG);
            turret.aimAt(0);
            return;
        }
        if (newFrame) seeing = camera.hasTarget() && camera.getLatencyMs() <= MAX_FRAME_AGE_MS;
        if (newFrame && seeing) {
            // Use where the turret and robot were when the picture was taken, so turning since then isn't counted twice
            int i = lookup(now - (long) (camera.getLatencyMs() * 1e6));
            // Turn until the shooter points at the tag average (aim table on hold, not used for aiming)
            offsetDeg = shooterOffsetDeg(turret.goalDistance(robot));
            double raw = histAngle[i] + (AIM_SIGN * camera.getTx() - offsetDeg) + turret.aimCorrection(robot);
            if (lastGood == Long.MIN_VALUE) {
                base = raw; // fresh lock: nothing to smooth from
            } else {
                double prev = base + TurretTrack.normalize(baseHeading - histHeading[i]); // old target, moved to this frame's heading
                base = prev + SMOOTHING * (raw - prev);
            }
            baseHeading = histHeading[i];
            lastGood = now;
        }
        if (lastGood != Long.MIN_VALUE && (now - lastGood) / 1e6 <= GRACE_MS) {
            mode = seeing ? Mode.CAMERA : Mode.GRACE;
            turret.setDeadband(deadband);
            // Robot turned left by d since the picture -> turret turns right by d to stay on the target
            turret.aimAt(base + TurretTrack.normalize(baseHeading - heading)); // clamps to the soft limits
        } else {
            mode = Mode.ODOMETRY;
            lastGood = Long.MIN_VALUE;
            turret.setDeadband(TurretTrack.DEADBAND_DEG);
            turret.update(robot, true); // odometry aim, soft limits inside TurretTrack
        }
    }

    /** Driver open-loop power (+ = left). Soft limits and the encoder-fault stop still apply (TurretTrack.manual). */
    public void manual(double p) {
        mode = Mode.MANUAL;
        lastGood = Long.MIN_VALUE;
        seeing = false;
        histCount = 0; // history isn't recorded here, so drop it: a camera lock after manual must not use stale angles
        turret.manual(p);
    }

    /** Degrees to turn right so the shooter, not the camera, points at the tags, at this distance (in) to the goal. */
    public static double shooterOffsetDeg(double distanceIn) {
        return Math.toDegrees(Math.atan2(CAMERA_RIGHT_OF_SHOOTER_IN, Math.max(MIN_OFFSET_DISTANCE_IN, distanceIn)));
    }

    private void record(long t, double angle, double heading) {
        histHead = (histHead + 1) % HISTORY;
        histTime[histHead] = t;
        histAngle[histHead] = angle;
        histHeading[histHead] = heading;
        histCount = Math.min(histCount + 1, HISTORY);
    }

    /** Index of the newest entry at or before t (the oldest one if t is older than the history). */
    private int lookup(long t) {
        int i = histHead;
        for (int n = 1; n < histCount && histTime[i] > t; n++) i = (i - 1 + HISTORY) % HISTORY;
        return i;
    }

    public Mode getMode() { return mode; }
    /** Last camera-to-shooter correction (deg, turret turned right by this). */
    public double getShooterOffsetDeg() { return offsetDeg; }
    /** Aim table at this pose: {aim correction deg, shooter tx deg, expected ty deg}. */
    public double[] aimTable(Pose robot) { return turret.aimTable(robot); }
    /** Measured aim correction at this pose (deg, + = left), added in both camera and odometry modes. */
    public double getAimCorrection(Pose robot) { return turret.aimCorrection(robot); }
    public TurretTrack.Alliance getAlliance() { return turret.getAlliance(); }
    public double getAngleDeg() { return turret.getAngleDeg(); }
    public double getTarget() { return turret.getTarget(); } // after the soft-limit clamp
    public double getGoalX() { return turret.getGoalX(); }
    public double getGoalY() { return turret.getGoalY(); }
    public double getError() { return turret.getError(); }
    public double getPower() { return turret.getPower(); }
    public boolean isAtLimit() { return turret.isAtLimit(); }
    public boolean isEncoderFault() { return turret.isEncoderFault(); }
    /** Camera-mode deadband (live tuning). Odometry keeps TurretTrack.DEADBAND_DEG. */
    public double getDeadband() { return deadband; }
    public void setDeadband(double deg) { deadband = Math.max(0, deg); }
    public HiveTagCamera getCamera() { return camera; }
}

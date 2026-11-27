package org.firstinspires.ftc.teamcode.subsytems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.HiveTagCamera;
import org.firstinspires.ftc.teamcode.vision.VisionPose;

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

    // Odometry correction from the camera's field pose (see VisionPose). Off until the field map, frame conversion and
    // camera mount are checked in Test Vision Pose; the estimate is still computed and shown while off.
    public static final boolean POSE_CORRECTION_ENABLED = false;
    public static final int MIN_POSE_TAGS = 2; // one tag gives a jumpy pose
    public static final double MAX_POSE_JUMP_IN = 12; // a bigger gap to odometry is a bad frame (wrong map, tags), not drift
    public static final double MAX_HEADING_ERROR_DEG = 10; // vision heading vs odometry heading + turret: sanity check
    public static final double FIELD_MARGIN_IN = 6; // reject poses this far outside the 144 in field
    public static final double POSE_BLEND = 0.2; // fraction of the x/y error corrected per accepted frame
    public static final double MIN_POSE_APPLY_IN = 0.25; // corrections add up until they reach this, then odometry is moved

    private static final int HISTORY = 256; // > 0.5 s of loops
    private final long[] histTime = new long[HISTORY];
    private final double[] histAngle = new double[HISTORY], histHeading = new double[HISTORY];
    private final double[] histX = new double[HISTORY], histY = new double[HISTORY];
    private int histCount, histHead;

    private final TurretTrack turret;
    private final HiveTagCamera camera;
    private Mode mode = Mode.ODOMETRY;
    private double deadband = DEADBAND_DEG; // camera modes only; odometry uses TurretTrack.DEADBAND_DEG
    private double base, baseHeading; // base = turret target at capture, baseHeading = robot heading then (deg)
    private long lastGood = Long.MIN_VALUE;
    private boolean seeing; // latest frame had usable tags
    private double offsetDeg; // last shooter offset used
    private boolean poseCorrection = POSE_CORRECTION_ENABLED;
    private long poseEpoch = Long.MIN_VALUE; // pictures taken before this predate the last pose reset
    private double pendingDx, pendingDy; // accepted correction not yet taken by consumePoseCorrection
    private Pose visionPose; // latest robot pose estimate from the camera, null = none
    private double visionDx, visionDy, visionHeadingErr; // estimate minus odometry at capture time
    private boolean visionAccepted;

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
        pendingDx = pendingDy = 0;
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
        record(now, angle, heading, robot.x(), robot.y());
        // Camera uses only the cell of the goal odometry picks (nearer, with hysteresis), so both modes aim at the same goal.
        // goals[0] (3-nectar) is the second tag group on both alliances (red 34-37, blue 42-45), so cell = 1 - goal.
        boolean newFrame = camera.update(turret.getAlliance() == TurretTrack.Alliance.RED, 1 - turret.pickGoal(robot));
        if (newFrame) estimatePose(now);

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

    private void record(long t, double angle, double heading, double x, double y) {
        histHead = (histHead + 1) % HISTORY;
        histTime[histHead] = t;
        histAngle[histHead] = angle;
        histHeading[histHead] = heading;
        histX[histHead] = x;
        histY[histHead] = y;
        histCount = Math.min(histCount + 1, HISTORY);
    }

    /**
     * Robot pose from the camera's field pose, compared with where odometry was when the picture was taken. A frame that
     * passes the checks moves odometry a fraction (POSE_BLEND) of the x/y error. Heading is never corrected: odometry
     * heading drifts less, and it places the camera offset.
     */
    private void estimatePose(long now) {
        visionPose = null;
        visionAccepted = false;
        long capture = now - (long) (camera.getLatencyMs() * 1e6);
        if (!camera.hasFieldPose() || camera.getFieldTagCount() < MIN_POSE_TAGS || histCount == 0
                || camera.getLatencyMs() > MAX_FRAME_AGE_MS || capture < poseEpoch) return;
        int i = lookup(capture);
        Pose v = VisionPose.robotFromCamera(camera.getFieldX(), camera.getFieldY(), camera.getFieldYawDeg(),
                histAngle[i], Math.toRadians(histHeading[i]));
        visionPose = v;
        visionDx = v.x() - histX[i];
        visionDy = v.y() - histY[i];
        visionHeadingErr = TurretTrack.normalize(Math.toDegrees(v.heading()) - histHeading[i]);
        double lo = -FIELD_MARGIN_IN, hi = 144 + FIELD_MARGIN_IN;
        visionAccepted = Math.abs(visionHeadingErr) <= MAX_HEADING_ERROR_DEG
                && Math.hypot(visionDx, visionDy) <= MAX_POSE_JUMP_IN
                && v.x() >= lo && v.x() <= hi && v.y() >= lo && v.y() <= hi;
        if (visionAccepted && poseCorrection) {
            double cx = POSE_BLEND * visionDx, cy = POSE_BLEND * visionDy;
            pendingDx += cx;
            pendingDy += cy;
            // The caller moves odometry by this, so move the stored poses too or the next frame counts it again
            for (int n = 0; n < histCount; n++) {
                int k = (histHead - n + HISTORY) % HISTORY;
                histX[k] += cx;
                histY[k] += cy;
            }
        }
    }

    /**
     * Odometry correction (x/y only, heading 0) accepted since the last call, or null. The caller must apply it:
     * localizer.setPose(pose + this) in TeleOp, follower.setPose(...) in autos. Call every loop after update().
     */
    public Pose consumePoseCorrection() {
        if (Math.hypot(pendingDx, pendingDy) < MIN_POSE_APPLY_IN) return null; // setPose is a hardware write: batch tiny fixes
        Pose c = new Pose(pendingDx, pendingDy, 0);
        pendingDx = pendingDy = 0;
        return c;
    }

    /** Call after setting odometry to a known pose by hand: older pictures are compared with a pose that no longer exists. */
    public void poseWasReset() {
        poseEpoch = System.nanoTime();
        pendingDx = pendingDy = 0;
    }

    /** Index of the newest entry at or before t (the oldest one if t is older than the history). */
    private int lookup(long t) {
        int i = histHead;
        for (int n = 1; n < histCount && histTime[i] > t; n++) i = (i - 1 + HISTORY) % HISTORY;
        return i;
    }

    public Mode getMode() { return mode; }
    public boolean isPoseCorrection() { return poseCorrection; }
    public void setPoseCorrection(boolean on) { poseCorrection = on; }
    /** Robot pose the camera implies for the latest usable frame (Pedro frame), or null. Heading is vision's own. */
    public Pose getVisionPose() { return visionPose; }
    /** Vision minus odometry at capture time: x, y (in) and heading (deg, vision heading from camera yaw less turret). */
    public double getVisionDx() { return visionDx; }
    public double getVisionDy() { return visionDy; }
    public double getVisionHeadingError() { return visionHeadingErr; }
    /** Latest estimate passed the checks (so it is applied when correction is on). */
    public boolean isVisionAccepted() { return visionAccepted; }
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

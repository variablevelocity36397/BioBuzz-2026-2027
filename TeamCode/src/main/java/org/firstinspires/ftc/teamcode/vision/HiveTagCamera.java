package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Limelight hive-tag finder. Picks our alliance's cell and reports the average tx/ty of its visible tags. */
public class HiveTagCamera {
    public static final int PIPELINE = 6;
    // BIOBUZZ hive tags: red cells 30-33, 34-37; blue cells 38-41, 42-45
    public static final int FIRST_TAG = 30;
    public static final int TAGS_PER_CELL = 4;

    private final Limelight3A limelight;
    private double tx, ty, targetArea, latencyMs;
    private String tagIds = "";
    private long frameAgeMs;
    private int tagCount;
    private double lastFrameTime = Double.NaN;

    public HiveTagCamera(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    /**
     * Reads the latest frame. Returns true only for a new frame, so the caller never applies one frame twice.
     * Only the given cell is averaged (averaging both would aim at the wall between them). The caller picks it, so the
     * far cell's tags seen through the hive are never used. cell 0 = first 4 tags (red 30-33, blue 38-41), 1 = next 4.
     */
    public boolean update(boolean red, int cell) {
        LLResult result = limelight.getLatestResult();
        if (result == null) return false;
        frameAgeMs = result.getStaleness();
        if (result.getTimestamp() == lastFrameTime) return false;
        lastFrameTime = result.getTimestamp();
        // How long ago the picture was taken: exposure + processing + time since it reached the hub
        latencyMs = frameAgeMs + result.getCaptureLatency() + result.getTargetingLatency();

        int first = red ? FIRST_TAG : FIRST_TAG + 2 * TAGS_PER_CELL; // each alliance has 2 cells
        int[] count = new int[2];
        double[] sumTx = new double[2], sumTy = new double[2], area = new double[2];
        String[] ids = {"", ""};
        if (result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id < first || id >= first + 2 * TAGS_PER_CELL) continue;
                int c = (id - first) / TAGS_PER_CELL;
                count[c]++;
                sumTx[c] += tag.getTargetXDegrees();
                sumTy[c] += tag.getTargetYDegrees();
                ids[c] += id + " ";
                area[c] += tag.getTargetArea();
            }
        }
        tagCount = count[cell];
        tagIds = ids[cell];
        targetArea = area[cell];
        if (!hasTarget()) {
            tx = ty = 0; // don't show a stale tx
        } else {
            tx = sumTx[cell] / count[cell];
            ty = sumTy[cell] / count[cell];
        }
        return true;
    }

    public boolean hasTarget() { return tagCount > 0; }
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public int getTagCount() { return tagCount; }
    /** IDs of the tags seen in the cell being aimed at. */
    public String getTagIds() { return tagIds; }
    public double getTargetArea() { return targetArea; }
    /** How old the latest Limelight frame is when it reached the hub. */
    public long getFrameAgeMs() { return frameAgeMs; }
    /** How long ago the latest frame's picture was taken (ms). */
    public double getLatencyMs() { return latencyMs; }
}

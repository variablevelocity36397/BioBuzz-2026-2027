package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

// Limelight that only sees AprilTags, it never moves the robot
public class LimeLightCamera {
    public enum Alliance { RED, BLUE }

    private Limelight3A limelight;
    private LLResult latest = null;  // result for this loop, set by update()

    private static final String CAMERA_NAME = "limelight";  // must match the robot config
    private static final int APRILTAG_PIPELINE = 3;

    // Outermost 2 tags of each CELL's 4-tag cluster (BioBuzz manual Fig 9-17), verify on robot
    private static final int[][] RED_CELLS = { {30, 33}, {34, 37} };
    private static final int[][] BLUE_CELLS = { {38, 41}, {42, 45} };
    private int[][] targetCells = null;  // null until an alliance is chosen, so no target is reported

    public LimeLightCamera(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, CAMERA_NAME);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    // Call once at the top of every loop so all getters use the same frame
    public void update() {
        latest = limelight.getLatestResult();
    }

    public void setAlliance(Alliance alliance) {
        if (alliance == Alliance.RED) {
            targetCells = RED_CELLS;
        } else {
            targetCells = BLUE_CELLS;
        }
    }

    private boolean resultValid() {
        return latest != null && latest.isValid();
    }

    // tx to aim at, or Double.NaN if none of our tags are visible
    public double getTargetTx() {
        if (targetCells == null || !resultValid()) {
            return Double.NaN;
        }

        double best = Double.NaN;
        for (int[] cell : targetCells) {
            double sum = 0;
            int count = 0;
            for (LLResultTypes.FiducialResult tag : latest.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id == cell[0] || id == cell[1]) {
                    sum += tag.getTargetXDegrees();
                    count++;
                }
            }
            if (count > 0) {
                double cellTx = sum / count;  // 2 seen -> average (CELL center), 1 seen -> that tag's tx
                // if both CELLs are visible, use the one closest to the crosshair
                if (Double.isNaN(best) || Math.abs(cellTx) < Math.abs(best)) {
                    best = cellTx;
                }
            }
        }
        return best;
    }

    public boolean hasTarget() {
        return !Double.isNaN(getTargetTx());
    }

    // Same as getTargetTx() but 0 when nothing is seen, for telemetry
    public double getTx() {
        double tx = getTargetTx();
        if (Double.isNaN(tx)) {
            return 0;
        }
        return tx;
    }

    // Every tag ID in view, before any alliance filtering
    public List<Integer> getVisibleTagIds() {
        List<Integer> ids = new ArrayList<>();
        if (!resultValid()) {
            return ids;
        }
        for (LLResultTypes.FiducialResult tag : latest.getFiducialResults()) {
            ids.add(tag.getFiducialId());
        }
        return ids;
    }

    // Raw Limelight data for debugging
    public void addDebugTelemetry(Telemetry telemetry) {
        telemetry.addData("LL connected", limelight.isConnected());
        telemetry.addData("LL running", limelight.isRunning());
        telemetry.addData("LL ms since update", limelight.getTimeSinceLastUpdate());
        telemetry.addData("LL valid", resultValid());
        if (latest != null) {
            telemetry.addData("LL pipeline", latest.getPipelineIndex());
            telemetry.addData("LL staleness ms", latest.getStaleness());
        }
        telemetry.addData("LL tag IDs seen", getVisibleTagIds());
    }

    public void stop() {
        limelight.stop();
    }
}

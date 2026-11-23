package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class BallDetectionProcessor implements VisionProcessor {

    // =========================================================
    // TUNING CONSTANTS
    // =========================================================
    // Adjust these HSV values to match your ball's color (OpenCV H is 0-180)
    public static double H_LOW = 20, S_LOW = 100, V_LOW = 100;   // Example: Yellow
    public static double H_HIGH = 30, S_HIGH = 255, V_HIGH = 255;

    /** Minimum contour area (px, in 320x240 space) to be considered a ball. */
    public static double MIN_AREA = 150;
    /** 4*pi*A/P^2 : 1.0 = perfect circle. Rejects tape, edges, rectangles. */
    public static double MIN_CIRCULARITY = 0.6;

    public static final int PROCESS_WIDTH = 320;
    public static final int PROCESS_HEIGHT = 240;

    // Immutable snapshot class to prevent thread-safety race conditions (torn reads)
    public static class BallPosition {
        public static final BallPosition NOT_DETECTED = new BallPosition(false, -1, -1, 0, 0, 0);

        public final boolean detected;
        public final double x;       // centroid X, 320x240 space
        public final double y;       // centroid Y, 320x240 space
        public final double radius;  // enclosing circle radius, 320x240 space
        public final double area;    // contour area, 320x240 space
        public final long timestampNanos; // System.nanoTime() when this frame was processed

        public BallPosition(boolean detected, double x, double y, double radius, double area, long timestampNanos) {
            this.detected = detected;
            this.x = x;
            this.y = y;
            this.radius = radius;
            this.area = area;
            this.timestampNanos = timestampNanos;
        }

        /** Age of this detection in milliseconds. */
        public double ageMs() {
            return (System.nanoTime() - timestampNanos) / 1e6;
        }
    }

    // OpenCV matrices declared once at the class level to prevent memory leaks
    private final Mat workingMat = new Mat();
    private final Mat blurredMat = new Mat();
    private final Mat hsvMat = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));

    // Target 320x240 size
    private final Size targetSize = new Size(PROCESS_WIDTH, PROCESS_HEIGHT);
    private final Size blurSize = new Size(5, 5);

    private final Scalar lowerBound = new Scalar(0, 0, 0);
    private final Scalar upperBound = new Scalar(0, 0, 0);

    // Single volatile reference guarantees atomic updates and reads across threads
    private volatile BallPosition currentPosition = BallPosition.NOT_DETECTED;

    private volatile double[] centerHsv = null;
    private volatile double[] centerRaw = null;
    private volatile int centerChannels = 0;
    private volatile double[] blobHsv = null;

    private final Paint circlePaint = new Paint();
    private final Paint centerPaint = new Paint();
    private final Paint crossPaint = new Paint();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        circlePaint.setColor(Color.GREEN);
        circlePaint.setStyle(Paint.Style.STROKE);
        circlePaint.setAntiAlias(true);
        centerPaint.setColor(Color.RED);
        centerPaint.setStyle(Paint.Style.FILL);
        centerPaint.setAntiAlias(true);
        crossPaint.setColor(Color.WHITE);
        crossPaint.setStyle(Paint.Style.STROKE);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long processedAt = System.nanoTime(); // same clock as ageMs()

        // 1. DECIMATE / ENSURE 320x240 RESOLUTION WITHOUT ALIASING MEMORY
        Mat inputMat = frame;
        if (frame.cols() != PROCESS_WIDTH || frame.rows() != PROCESS_HEIGHT) {
            Imgproc.resize(frame, workingMat, targetSize, 0, 0, Imgproc.INTER_AREA);
            inputMat = workingMat;
        }
        // Scale factor to map 320x240 coordinates back onto the original frame for drawing
        double scaleX = frame.cols() / (double) PROCESS_WIDTH;
        double scaleY = frame.rows() / (double) PROCESS_HEIGHT;

        // 2. BLUR IN RGB FIRST, THEN CONVERT TO HSV
        Imgproc.GaussianBlur(inputMat, blurredMat, blurSize, 0);
        Imgproc.cvtColor(blurredMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        // 3. THRESHOLD COLOR (bounds re-read each frame so static tuning values apply live)
        lowerBound.set(new double[]{H_LOW, S_LOW, V_LOW});
        upperBound.set(new double[]{H_HIGH, S_HIGH, V_HIGH});
        Core.inRange(hsvMat, lowerBound, upperBound, mask);

        // 4. CLEAN MASK: open removes speckle, close fills glare holes in the ball
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, morphKernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, morphKernel);

        // 5. FIND CONTOURS
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double bestArea = 0;
        MatOfPoint bestContour = null;
        double bestRadius = 0;

        MatOfPoint2f contour2f = new MatOfPoint2f();
        Point circleCenter = new Point();
        float[] circleRadius = new float[1];

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < MIN_AREA || area <= bestArea) continue;

            // Shape filter: reject non-round blobs (tape, field elements, etc.)
            contour.convertTo(contour2f, CvType.CV_32F);
            double perimeter = Imgproc.arcLength(contour2f, true);
            if (perimeter <= 0) continue;
            double circularity = 4 * Math.PI * area / (perimeter * perimeter);
            if (circularity < MIN_CIRCULARITY) continue;

            Imgproc.minEnclosingCircle(contour2f, circleCenter, circleRadius);
            bestArea = area;
            bestContour = contour;
            bestRadius = circleRadius[0];
        }
        contour2f.release();

        // 6. CALCULATE CENTER COORDINATES ATOMICALLY
        BallPosition result = BallPosition.NOT_DETECTED;
        if (bestContour != null) {
            Moments moments = Imgproc.moments(bestContour);
            if (moments.get_m00() != 0) {
                double centerX = moments.get_m10() / moments.get_m00();
                double centerY = moments.get_m01() / moments.get_m00();
                result = new BallPosition(true, centerX, centerY, bestRadius, bestArea, processedAt);
            }
        }
        if (!result.detected) {
            // Still timestamp "not detected" so the OpMode can tell the camera is alive
            result = new BallPosition(false, -1, -1, 0, 0, processedAt);
        }
        currentPosition = result;

        // Diagnostics: HSV of the frame center and of the detected blob's centroid,
        // so the real ball / false-target colors can be read off in telemetry.
        centerHsv = pixelHsv((int) (PROCESS_HEIGHT / 2.0), (int) (PROCESS_WIDTH / 2.0));
        int cr = (int) (PROCESS_HEIGHT / 2.0), cc = (int) (PROCESS_WIDTH / 2.0);
        centerRaw = (cr < inputMat.rows() && cc < inputMat.cols()) ? inputMat.get(cr, cc) : null;
        centerChannels = inputMat.channels();
        blobHsv = result.detected ? pixelHsv((int) result.y, (int) result.x) : null;

        // Release contour native memory
        for (MatOfPoint contour : contours) {
            contour.release();
        }

        // Pass a draw snapshot (original-frame pixel coords) to onDrawFrame
        if (result.detected) {
            return new double[]{result.x * scaleX, result.y * scaleY, result.radius * (scaleX + scaleY) / 2.0};
        }
        return new double[0];
    }

    private double[] pixelHsv(int row, int col) {
        if (row < 0 || col < 0 || row >= hsvMat.rows() || col >= hsvMat.cols()) return null;
        return hsvMat.get(row, col);
    }

    /** Raw channel values of the frame center pixel as delivered by the camera (before HSV), or null. */
    public double[] getCenterRaw() { return centerRaw; }

    /** Number of channels in the incoming frame (3 = RGB, 4 = RGBA). */
    public int getChannels() { return centerChannels; }

    /** HSV (H 0-180, S 0-255, V 0-255) of the frame center pixel, or null. */
    public double[] getCenterHsv() { return centerHsv; }

    /** HSV of the detected blob's centroid pixel, or null if nothing detected. */
    public double[] getBlobHsv() { return blobHsv; }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Crosshair at frame center: aim the ball here and read "Center HSV" in telemetry
        float mx = onscreenWidth / 2f, my = onscreenHeight / 2f;
        float arm = 10 * scaleCanvasDensity;
        crossPaint.setStrokeWidth(2 * scaleCanvasDensity);
        canvas.drawLine(mx - arm, my, mx + arm, my, crossPaint);
        canvas.drawLine(mx, my - arm, mx, my + arm, crossPaint);

        if (!(userContext instanceof double[])) return;
        double[] d = (double[]) userContext;
        if (d.length < 3) return;
        float cx = (float) (d[0] * scaleBmpPxToCanvasPx);
        float cy = (float) (d[1] * scaleBmpPxToCanvasPx);
        float r = (float) (d[2] * scaleBmpPxToCanvasPx);

        circlePaint.setStrokeWidth(3 * scaleCanvasDensity);
        canvas.drawCircle(cx, cy, r, circlePaint);
        canvas.drawCircle(cx, cy, 4 * scaleCanvasDensity, centerPaint);
    }

    // Atomic getter for the OpMode to read
    public BallPosition getBallPosition() {
        return currentPosition;
    }
}
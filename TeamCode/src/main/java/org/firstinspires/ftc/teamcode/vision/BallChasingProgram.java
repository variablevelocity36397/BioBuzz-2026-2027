package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Ball Tracker and Chase", group = "Vision")
public class BallChasingProgram extends LinearOpMode {

    // =========================================================
    // TUNING CONSTANTS (For 320x240 effective resolution)
    // =========================================================
    public static double KP_TURN = 0.002;
    public static double KP_FORWARD = 0.0025;
    public static double DEADBAND_X = 25.0;
    public static double DEADBAND_Y = 25.0;

    // Static friction feedforward & max speed limits
    public static double MIN_POWER = 0.10;
    public static double MAX_POWER = 0.30;

    // Lost-target behavior
    public static double HOLD_LAST_MS = 100;     // brief dropout bridge (coasting/overshoot risk if long)
    public static double STALE_FRAME_MS = 250;   // ignore detections older than this (camera froze)
    public static double SEARCH_TURN_POWER = 0.0; // set e.g. 0.2 to rotate and search when lost; 0 = stop

    // Camera white balance: 0 = automatic. If the image has a color tint, try a manual
    // value such as 4500 (warm/indoor) to 6500 (daylight) until colors look natural.
    public static double WB_TEMPERATURE_K = 0;

    // 320x240 resolution coordinates
    public static final double FRAME_WIDTH = BallDetectionProcessor.PROCESS_WIDTH;
    public static final double CAMERA_CENTER_X = FRAME_WIDTH / 2.0;
    public static double CAMERA_X_OFFSET = 0.0;   // px; shift if camera is not centered on the robot
    public static double TARGET_Y = 190.0;        // Target near bottom of frame when ball is close

    // Camera is yawed relative to the robot's forward direction.
    // Positive = camera points to the RIGHT of robot forward, negative = LEFT. Flip sign if it drives the wrong way.
    public static double CAMERA_YAW_DEG = 45.0;

    private enum State { TRACKING, HOLD, SEARCH }

    private BallDetectionProcessor ballDetectionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        MecanumDrive robotDrive = new MecanumDrive();
        robotDrive.init(hardwareMap);

        ballDetectionProcessor = new BallDetectionProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(ballDetectionProcessor)
                .build();

        // Camera color fix: the webcam's own white balance can drift (e.g. a red tint).
        // Must be applied once the camera is streaming.
        configureCameraColor();

        // Live detection preview during init so HSV / circularity can be tuned before Start
        while (opModeInInit()) {
            BallDetectionProcessor.BallPosition ball = ballDetectionProcessor.getBallPosition();
            telemetry.addData("Status", "Initialized at 320x240. Waiting for Start.");
            addBallTelemetry(ball);
            telemetry.addData("Camera FPS", "%.1f", visionPortal.getFps());
            telemetry.update();
            sleep(50);
        }

        State state = State.SEARCH;
        ElapsedTime lostTimer = new ElapsedTime();
        double lastForward = 0.0;
        double lastRotate = 0.0;

        while (opModeIsActive()) {

            // 1. GET ATOMIC SNAPSHOT FROM THE PROCESSOR
            BallDetectionProcessor.BallPosition ball = ballDetectionProcessor.getBallPosition();
            boolean fresh = ball.ageMs() < STALE_FRAME_MS;
            boolean valid = ball.detected && fresh;

            double forwardPower;
            double rotatePower;
            double strafePower = 0.0;

            if (valid) {
                state = State.TRACKING;
                lostTimer.reset();

                // 2. CALCULATE ERROR
                double errorX = ball.x - (CAMERA_CENTER_X + CAMERA_X_OFFSET);
                double errorY = TARGET_Y - ball.y;

                // 3. P CONTROL + STATIC FEEDFORWARD (P term starts at 0 at the deadband edge)
                rotatePower = control(errorX, DEADBAND_X, KP_TURN, MIN_POWER, MAX_POWER);
                forwardPower = control(errorY, DEADBAND_Y, KP_FORWARD, MIN_POWER, MAX_POWER);

                // 4. TURN FIRST, THEN COMMIT: scale forward down when badly misaligned
                double alignment = Math.max(0.0, 1.0 - Math.abs(errorX) / CAMERA_CENTER_X);
                forwardPower *= alignment;

                // 5. CAMERA IS YAWED: the ball is kept centered in the camera, so the ball lies along
                // the camera axis. Split the approach speed into robot forward + strafe components.
                double yaw = Math.toRadians(CAMERA_YAW_DEG);
                double approach = forwardPower;
                forwardPower = approach * Math.cos(yaw);
                strafePower = approach * Math.sin(yaw);

                lastForward = forwardPower;
                lastRotate = rotatePower;

                telemetry.addData("Error X", "%.1f", errorX);
                telemetry.addData("Error Y", "%.1f", errorY);
            } else if (state != State.SEARCH && lostTimer.milliseconds() < HOLD_LAST_MS) {
                // Brief dropout: keep the last command to avoid stutter
                state = State.HOLD;
                forwardPower = 0.0; // do not keep driving blind: latency causes overshoot
                rotatePower = 0.0;
            } else {
                // Lost: stop driving forward, optionally rotate to search
                state = State.SEARCH;
                forwardPower = 0.0;
                rotatePower = SEARCH_TURN_POWER;
                lastForward = 0.0;
                lastRotate = 0.0;
            }

            // 5. APPLY POWER TO DRIVETRAIN
            robotDrive.drive(forwardPower, strafePower, rotatePower);

            telemetry.addData("State", state);
            if (!fresh) telemetry.addData("WARNING", "Stale camera data (%.0f ms)", ball.ageMs());
            addBallTelemetry(ball);
            telemetry.addData("Motor Power - Forward", "%.2f", forwardPower);
            telemetry.addData("Motor Power - Turn", "%.2f", rotatePower);
            telemetry.addData("Camera FPS", "%.1f", visionPortal.getFps());
            telemetry.update();
        }
        robotDrive.drive(0.0, 0.0, 0.0);
        visionPortal.close();
    }

    /**
     * Proportional control with a deadband and static-friction feedforward.
     * Outside the deadband, output = kS + kP*(|error| - deadband), so the P term starts at 0
     * and only the kS (MIN_POWER) step remains at the deadband edge.
     */
    static double control(double error, double deadband, double kP, double kS, double max) {
        double absErr = Math.abs(error);
        if (absErr <= deadband) return 0.0;
        double magnitude = kP * (absErr - deadband) + kS;
        return Math.copySign(Math.min(magnitude, max), error);
    }

    private void configureCameraColor() {
        ElapsedTime wait = new ElapsedTime();
        while (!isStopRequested()
                && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING
                && wait.seconds() < 4.0) {
            sleep(20);
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Not streaming; skipped color setup");
            return;
        }
        try {
            WhiteBalanceControl wb = visionPortal.getCameraControl(WhiteBalanceControl.class);
            if (wb == null) {
                telemetry.addData("White balance", "control not available");
            } else if (WB_TEMPERATURE_K <= 0) {
                wb.setMode(WhiteBalanceControl.Mode.AUTO);
                telemetry.addData("White balance", "AUTO");
            } else {
                int k = (int) Range.clip(WB_TEMPERATURE_K,
                        wb.getMinWhiteBalanceTemperature(), wb.getMaxWhiteBalanceTemperature());
                wb.setMode(WhiteBalanceControl.Mode.MANUAL);
                wb.setWhiteBalanceTemperature(k);
                telemetry.addData("White balance", "MANUAL %d K", k);
            }
        } catch (Exception e) {
            telemetry.addData("White balance", "setup failed: " + e.getMessage());
        }
    }

    private void addBallTelemetry(BallDetectionProcessor.BallPosition ball) {
        if (ball.detected) {
            telemetry.addData("Ball", "x=%.1f  y=%.1f  r=%.1f  area=%.0f",
                    ball.x, ball.y, ball.radius, ball.area);
        } else {
            telemetry.addData("Ball", "Not Detected");
        }
        telemetry.addData("Center HSV", fmtHsv(ballDetectionProcessor.getCenterHsv()));
        double[] raw = ballDetectionProcessor.getCenterRaw();
        telemetry.addData("Center raw (" + ballDetectionProcessor.getChannels() + " ch)",
                raw == null ? "n/a" : java.util.Arrays.toString(raw));
        telemetry.addData("Detected-blob HSV", fmtHsv(ballDetectionProcessor.getBlobHsv()));
    }

    private static String fmtHsv(double[] hsv) {
        if (hsv == null || hsv.length < 3) return "n/a";
        return String.format("H=%.0f S=%.0f V=%.0f", hsv[0], hsv[1], hsv[2]);
    }
}
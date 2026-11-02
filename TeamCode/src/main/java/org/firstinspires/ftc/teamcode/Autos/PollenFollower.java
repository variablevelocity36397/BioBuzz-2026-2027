package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.skeletonarmy.marrow.BulkCaching;

/*
  POLLEN FOLLOWER - follow pollen only (no intake)
  ------------------------------------------------
  The robot does NOT use the intake. It:
    - TURNS toward the pollen when it moves left/right, to keep it in the middle of the camera
      (TRACK_MODE can switch this to sliding sideways, or both)
    - drives forward/backward only to stay a set distance from the ball (FOLLOW_WIDTH)
    - STOPS the moment it can't see the ball (no driving forward blind)
  Move the ball sideways -> the robot turns with it. Move it away -> it follows. Move it closer -> it backs up.

  Uses the same motor names and directions as "Pollen Collector (P)".
  If it turns the WRONG way, flip TURN_SIGN. Slides the wrong way: STRAFE_SIGN. Drives the wrong way: FORWARD_SIGN.
*/
@BulkCaching   // Marrow: faster hardware reads
@Autonomous(name = "Pollen Follower", group = "Pollen")
public class PollenFollower extends LinearOpMode {

    // ---------- Speed ----------
    private static final double SPEED = 0.5;              // multiplies every drive power (1.0 = full speed)

    // ---------- How it follows left/right movement ----------
    private enum Track { TURN, STRAFE, BOTH }
    private static final Track TRACK_MODE = Track.TURN;   // TURN = rotate to face the ball, STRAFE = slide, BOTH

    // ---------- Turning ----------
    private static final double KP_TURN      = 0.006;     // turn power per PIXEL the ball is off-center
    private static final double MAX_TURN     = 0.6;
    private static final double TURN_SIGN    = 1.0;       // set to -1.0 if it turns AWAY from the ball

    // ---------- Side to side (strafe, used in STRAFE / BOTH mode) ----------
    private static final double KP_STRAFE    = 0.008;     // slide power per PIXEL the ball is off-center
    private static final int    CENTER_DEADBAND = 12;     // pixels: this close to center counts as centered
    private static final double MAX_STRAFE   = 0.8;
    private static final double STRAFE_SIGN  = 1.0;       // set to -1.0 if it slides AWAY from the ball

    // ---------- Distance (forward / back) ----------
    private static final int    FOLLOW_WIDTH = 60;        // ball box this wide (px) = the distance to hold
    private static final int    WIDTH_DEADBAND = 6;       // pixels: close enough, don't move forward/back
    private static final double KP_DISTANCE  = 0.01;      // forward power per pixel of size error
    private static final double MAX_FORWARD  = 0.4;
    private static final double FORWARD_SIGN = 1.0;       // set to -1.0 if it backs away when it should approach

    // ---------- Camera ----------
    private static final int    POLLEN_ID = 1;            // HuskyLens learned ID for pollen
    private static final int    SCREEN_W  = 320;

    // Filters (only follow things that look like pollen)
    private static final int    MIN_BOX_PX     = 10;
    private static final double MIN_ASPECT     = 0.6, MAX_ASPECT = 1.6;
    private static final int    MIN_Y          = 60;      // ignore boxes near the top of the screen
    private static final int    CONFIRM_FRAMES = 2;       // must see it this many frames in a row

    private DcMotor fl, fr, bl, br;
    private HuskyLens huskyLens;

    private int seenFrames = 0;
    private String status = "Waiting for pollen";
    private HuskyLens.Block lastBall = null;
    private double lastStrafe = 0, lastForward = 0, lastTurn = 0;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl.setDirection(PollenCollector.FL_DIR);
        fr.setDirection(PollenCollector.FR_DIR);
        bl.setDirection(PollenCollector.BL_DIR);
        br.setDirection(PollenCollector.BR_DIR);
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        huskyLens = hardwareMap.get(HuskyLens.class, "husylens");   // your Driver Hub spelling
        boolean found = huskyLens.knock();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.addData("HuskyLens", found ? "ready" : "NOT FOUND - check wiring / config name 'husylens'");
        telemetry.addLine("Press START, then move a pollen ball in front of the camera.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block ball = findPollen();

            if (ball == null) {
                status = "No pollen - stopped";
                drive(0, 0, 0);
            } else {
                int xErr = ball.x - SCREEN_W / 2;
                boolean centered = Math.abs(xErr) <= CENTER_DEADBAND;

                // Ball moved right -> turn right (negative = clockwise). Ball moved left -> turn left.
                double turn = 0;
                if (!centered && TRACK_MODE != Track.STRAFE) {
                    turn = Range.clip(-KP_TURN * xErr, -MAX_TURN, MAX_TURN) * TURN_SIGN;
                }
                // Or slide sideways toward it
                double right = 0;
                if (!centered && TRACK_MODE != Track.TURN) {
                    right = Range.clip(KP_STRAFE * xErr, -MAX_STRAFE, MAX_STRAFE) * STRAFE_SIGN;
                }
                if (TRACK_MODE == Track.BOTH) { turn *= 0.5; right *= 0.5; }

                // Distance: ball smaller than FOLLOW_WIDTH -> too far -> drive forward (bigger -> back up)
                int wErr = FOLLOW_WIDTH - ball.width;
                double forward = Math.abs(wErr) <= WIDTH_DEADBAND ? 0 : KP_DISTANCE * wErr;
                // don't charge forward while the ball is way off to the side - line up first
                forward *= 1.0 - Range.clip(Math.abs(xErr) / (SCREEN_W / 2.0), 0, 1);
                forward = Range.clip(forward, -MAX_FORWARD, MAX_FORWARD) * FORWARD_SIGN;

                status = (right == 0 && forward == 0 && turn == 0) ? "Locked on - holding"
                        : (turn != 0 ? "Turning to follow" : "Following");
                drive(forward, right, turn);
            }
            show();
        }
        drive(0, 0, 0);
    }

    /** Biggest box that looks like pollen, after it's been seen CONFIRM_FRAMES in a row. */
    private HuskyLens.Block findPollen() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        HuskyLens.Block best = null;
        if (blocks != null) {
            for (HuskyLens.Block b : blocks) {
                if (b.id != POLLEN_ID) continue;
                if (b.width < MIN_BOX_PX || b.height < MIN_BOX_PX) continue;
                double aspect = (double) b.width / b.height;
                if (aspect < MIN_ASPECT || aspect > MAX_ASPECT) continue;
                if (b.y < MIN_Y) continue;
                if (best == null || b.width > best.width) best = b;
            }
        }
        if (best == null) { seenFrames = 0; return null; }
        lastBall = best;
        seenFrames++;
        return seenFrames >= CONFIRM_FRAMES ? best : null;
    }

    /** forward (+ = ahead), right (+ = slide right), turn (+ = turn left). Same mecanum math as the collector, times SPEED. */
    private void drive(double forward, double right, double turn) {
        lastForward = forward;
        lastStrafe = right;
        lastTurn = turn;
        double r = -turn;
        double lf = forward + right + r, lb = forward - right + r, rf = forward - right - r, rb = forward + right - r;
        double biggest = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb))));
        double k = SPEED / biggest;
        fl.setPower(lf * k); bl.setPower(lb * k); fr.setPower(rf * k); br.setPower(rb * k);
    }

    private void show() {
        telemetry.addData("Status", status);
        if (lastBall != null) {
            telemetry.addData("Ball box", "x %d (center 160)  w %d (hold at %d)",
                    lastBall.x, lastBall.width, FOLLOW_WIDTH);
        }
        telemetry.addData("Drive", "forward %.2f   turn left %.2f   slide right %.2f", lastForward, lastTurn, lastStrafe);
        telemetry.addData("Mode", TRACK_MODE);
        telemetry.update();
    }
}

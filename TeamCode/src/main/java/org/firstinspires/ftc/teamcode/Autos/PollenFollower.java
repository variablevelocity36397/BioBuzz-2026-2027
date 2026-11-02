package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.skeletonarmy.marrow.BulkCaching;

/*
  POLLEN FOLLOWER - follow the pollen and INTAKE it
  -------------------------------------------------
    0. SEARCH: no ball in view -> SPIN in place toward the side the ball was last seen, until one shows up.
    1. FOLLOW: LOCK ON to ONE ball (ignores other balls until this one is collected or lost),
               turn to face it (TRACK_MODE) and drive toward it.
    2. CLOSE:  when the ball is close, turn the INTAKE ON and keep driving straight at it.
    3. PASS:   the ball disappears under the camera before the intake - keep driving PASS_DISTANCE_IN
               inches past that point (wheel encoders) so the ball goes all the way in, then keep the
               intake on a moment longer.
    4. CHECK:  back up a little and look. If a ball is still right in front -> the pickup missed ->
               shift sideways a bit and try again (up to MAX_RETRIES). If nothing is there -> it's in.
    5. Go back to SEARCH for the next ball. Runs until STOP.

  Set COLLECT = false to only follow (no intake, holds FOLLOW_WIDTH distance).
  Uses the same motor names and directions as "Pollen Collector (P)".
  Spins the wrong way while searching: SEARCH_SIGN. Turns the wrong way: TURN_SIGN. Drives the wrong way: FORWARD_SIGN. Intake pushes out: flip
  INTAKE_MOTOR_DIR / LEFT_INTAKE_DIR / RIGHT_INTAKE_DIR in PollenCollector.java.
*/
@BulkCaching   // Marrow: faster hardware reads
@Autonomous(name = "Pollen Follower", group = "Pollen")
public class PollenFollower extends LinearOpMode {

    private static final boolean COLLECT = true;          // false = follow only, no intake

    // ---------- Speed ----------
    private static final double SPEED = 0.5;              // multiplies every drive power (1.0 = full speed)

    // ---------- Searching (no ball in view) ----------
    private static final double SEARCH_TURN  = 0.35;      // spin power while looking for a ball
    private static final double SEARCH_SIGN  = 1.0;       // -1.0 if it spins the opposite way you expect
    private static final double LOST_GRACE   = 0.4;       // seconds to wait after losing the ball before spinning

    // ---------- Locking onto ONE ball ----------
    private static final int    LOCK_JUMP_PX = 70;        // a box further than this from the locked ball = a DIFFERENT ball

    // ---------- How it follows left/right movement ----------
    private enum Track { TURN, STRAFE, BOTH }
    private static final Track TRACK_MODE = Track.TURN;   // TURN = rotate to face the ball, STRAFE = slide, BOTH
    private static final double KP_TURN      = 0.006;     // turn power per PIXEL the ball is off-center
    private static final double MAX_TURN     = 0.6;
    private static final double TURN_SIGN    = 1.0;       // -1.0 if it turns AWAY from the ball
    private static final double KP_STRAFE    = 0.008;
    private static final double MAX_STRAFE   = 0.8;
    private static final double STRAFE_SIGN  = 1.0;
    private static final int    CENTER_DEADBAND = 12;     // pixels: this close to center counts as centered

    // ---------- Driving toward the ball ----------
    private static final double APPROACH_POWER = 0.4;     // forward power when the ball is far
    private static final double MIN_APPROACH   = 0.25;    // forward power when it's getting close
    private static final double FORWARD_SIGN   = 1.0;     // -1.0 if it backs away when it should approach

    // ---------- Intake (tune with the "Ball box" telemetry) ----------
    private static final int    INTAKE_ZONE_WIDTH = 45;   // ball box this wide, OR...
    private static final int    INTAKE_ZONE_Y     = 170;  // ...this low on screen = CLOSE: intake on, drive at it
    private static final double ZONE_FORWARD      = 0.35; // forward power while driving onto the ball
    private static final double PASS_DISTANCE_IN  = 12.0; // keep driving this far after the ball vanishes under the camera
    private static final double PASS_POWER        = 0.35;
    private static final double PASS_TIMEOUT      = 2.5;  // safety: stop the pass after this many seconds
    private static final double INTAKE_EXTRA_TIME = 0.5;  // keep spinning after stopping so the ball settles in

    // ---------- Check + retry ----------
    private static final double BACKUP_IN   = 6.0;        // back up this far, then look for the ball
    private static final double LOOK_TIME   = 0.6;        // seconds to look after backing up
    private static final int    MISSED_WIDTH = 35;        // a ball this big right in front after backing up = we missed it
    private static final int    MAX_RETRIES = 2;
    private static final double RETRY_SHIFT_IN = 3.0;     // slide this far sideways before retrying

    // ---------- Follow-only distance (COLLECT = false) ----------
    private static final int    FOLLOW_WIDTH = 60, WIDTH_DEADBAND = 6;
    private static final double KP_DISTANCE = 0.01, MAX_FORWARD = 0.4;

    // ---------- Drive encoders (goBILDA 312 rpm Yellow Jacket, 104 mm mecanum) ----------
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_IN = 104.0 / 25.4;
    private static final double INCHES_PER_TICK = Math.PI * WHEEL_DIAMETER_IN / TICKS_PER_REV;

    // ---------- Camera ----------
    private static final int    POLLEN_ID = 1;
    private static final int    SCREEN_W  = 320;
    private static final int    MIN_BOX_PX = 10, MIN_Y = 60, CONFIRM_FRAMES = 2;
    private static final double MIN_ASPECT = 0.6, MAX_ASPECT = 1.6;

    private DcMotor fl, fr, bl, br, intakemotor;
    private CRServo leftintake, rightintake;
    private HuskyLens huskyLens;

    private int seenFrames = 0, collected = 0, missed = 0, retries = 0;
    private boolean wasClose = false;
    private int retrySide = 1;
    private String status = "Waiting for pollen";
    private HuskyLens.Block lastBall = null;
    private HuskyLens.Block lockedBall = null;  // the ONE ball we're chasing (null = not locked)
    private int lastSide = 1;                   // +1 = ball was last seen on the left, -1 = right (which way to spin)
    private final ElapsedTime lostTimer = new ElapsedTime();
    private double lastFwd = 0, lastTurn = 0, lastRight = 0;

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
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // we set power ourselves, but still read ticks
        }

        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        leftintake  = hardwareMap.get(CRServo.class, "leftintake");
        rightintake = hardwareMap.get(CRServo.class, "rightintake");
        intakemotor.setDirection(PollenCollector.INTAKE_MOTOR_DIR);
        leftintake.setDirection(PollenCollector.LEFT_INTAKE_DIR);
        rightintake.setDirection(PollenCollector.RIGHT_INTAKE_DIR);

        huskyLens = hardwareMap.get(HuskyLens.class, "husylens");   // your Driver Hub spelling
        boolean found = huskyLens.knock();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.addData("HuskyLens", found ? "ready" : "NOT FOUND - check wiring / config name 'husylens'");
        telemetry.addData("Mode", COLLECT ? "follow + INTAKE" : "follow only");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block ball = findPollen();

            if (ball == null) {
                if (COLLECT && wasClose) {
                    // It slid under the camera right in front of the intake -> finish the pickup
                    wasClose = false;
                    pickUpAndCheck();
                } else if (lockedBall != null && lostTimer.seconds() < LOST_GRACE) {
                    status = "Lost the ball - waiting a moment";
                    intake(false);
                    drive(0, 0, 0);
                } else {
                    // Nothing in view -> spin toward where we last saw a ball
                    lockedBall = null;
                    status = "Searching - spinning " + (lastSide > 0 ? "left" : "right");
                    intake(false);
                    drive(0, SEARCH_TURN * lastSide * SEARCH_SIGN, 0);
                }
                show();
                continue;
            }

            lockedBall = ball;                    // stay on THIS ball
            lostTimer.reset();
            int xErr = ball.x - SCREEN_W / 2;
            lastSide = xErr <= 0 ? 1 : -1;        // remember its side, so a search spins that way
            double[] side = sideways(xErr);       // {turn, right}
            boolean close = ball.width >= INTAKE_ZONE_WIDTH || ball.y >= INTAKE_ZONE_Y;

            if (!COLLECT) {
                int wErr = FOLLOW_WIDTH - ball.width;
                double fwd = Math.abs(wErr) <= WIDTH_DEADBAND ? 0 : KP_DISTANCE * wErr;
                fwd *= 1.0 - Range.clip(Math.abs(xErr) / (SCREEN_W / 2.0), 0, 1);
                status = "Following (no intake)";
                drive(Range.clip(fwd, -MAX_FORWARD, MAX_FORWARD) * FORWARD_SIGN, side[0], side[1]);
            } else if (close) {
                status = "CLOSE - intake ON, driving at it";
                wasClose = true;
                intake(true);
                drive(ZONE_FORWARD * FORWARD_SIGN, side[0] * 0.5, side[1] * 0.5);
            } else {
                status = "Following pollen";
                wasClose = false;
                intake(false);
                double closeness = Range.clip((double) ball.width / INTAKE_ZONE_WIDTH, 0, 1);
                double fwd = MIN_APPROACH + (APPROACH_POWER - MIN_APPROACH) * (1 - closeness);
                fwd *= 1.0 - 0.7 * Range.clip(Math.abs(xErr) / (SCREEN_W / 2.0), 0, 1);  // line up first
                drive(fwd * FORWARD_SIGN, side[0], side[1]);
            }
            show();
        }
        drive(0, 0, 0);
        intake(false);
    }

    /** Drive past the spot where the ball vanished, then back up and make sure we got it. Retry if not. */
    private void pickUpAndCheck() {
        lockedBall = null;   // the camera can't see it now; the check/retry below looks at whatever is in front
        for (int attempt = 0; opModeIsActive(); attempt++) {
            // PASS: keep going so the ball is pulled all the way in
            status = "Intaking - driving past the ball";
            intake(true);
            driveDistance(PASS_DISTANCE_IN, PASS_POWER, PASS_TIMEOUT);
            waitSeconds(INTAKE_EXTRA_TIME);          // intake still on
            intake(false);

            // CHECK: back up and look
            status = "Checking - did we get it?";
            driveDistance(-BACKUP_IN, PASS_POWER, PASS_TIMEOUT);
            HuskyLens.Block stillThere = lookFor(LOOK_TIME);

            if (stillThere == null || stillThere.width < MISSED_WIDTH) {
                collected++;
                status = "Got it! (" + collected + ")";
                return;
            }

            // MISSED: try again from a slightly different spot
            missed++;
            if (attempt >= MAX_RETRIES) {
                status = "Missed it " + (MAX_RETRIES + 1) + " times - giving up on this one";
                return;
            }
            retries++;
            status = "Missed - shifting sideways and trying again";
            strafeDistance(RETRY_SHIFT_IN * retrySide, PASS_TIMEOUT);
            retrySide = -retrySide;                  // next retry shifts the other way
            // re-approach: turn the intake on and drive at it while steering by the camera
            intake(true);
            ElapsedTime t = new ElapsedTime();
            while (opModeIsActive() && t.seconds() < PASS_TIMEOUT) {
                HuskyLens.Block b = findPollen();
                if (b == null) break;                // slid under the camera again -> pass
                double[] side = sideways(b.x - SCREEN_W / 2);
                drive(ZONE_FORWARD * FORWARD_SIGN, side[0] * 0.5, side[1] * 0.5);
                show();
            }
        }
    }

    /** {turn, right} to keep the ball centered, using TRACK_MODE. */
    private double[] sideways(int xErr) {
        if (Math.abs(xErr) <= CENTER_DEADBAND) return new double[]{0, 0};
        double turn = TRACK_MODE != Track.STRAFE ? Range.clip(-KP_TURN * xErr, -MAX_TURN, MAX_TURN) * TURN_SIGN : 0;
        double right = TRACK_MODE != Track.TURN ? Range.clip(KP_STRAFE * xErr, -MAX_STRAFE, MAX_STRAFE) * STRAFE_SIGN : 0;
        if (TRACK_MODE == Track.BOTH) { turn *= 0.5; right *= 0.5; }
        return new double[]{turn, right};
    }

    /** Drive straight forward (+) or back (-) a distance in inches using the wheel encoders. */
    private void driveDistance(double inches, double power, double timeout) {
        double start = avgForwardTicks();
        ElapsedTime t = new ElapsedTime();
        double dir = Math.signum(inches);
        while (opModeIsActive() && t.seconds() < timeout
                && Math.abs(avgForwardTicks() - start) * INCHES_PER_TICK < Math.abs(inches)) {
            drive(dir * power * FORWARD_SIGN, 0, 0);
            show();
        }
        drive(0, 0, 0);
    }

    /** Slide right (+) or left (-) a distance in inches (mecanum; roughly, from the encoders). */
    private void strafeDistance(double inches, double timeout) {
        double start = avgStrafeTicks();
        ElapsedTime t = new ElapsedTime();
        double dir = Math.signum(inches);
        while (opModeIsActive() && t.seconds() < timeout
                && Math.abs(avgStrafeTicks() - start) * INCHES_PER_TICK < Math.abs(inches)) {
            drive(0, 0, dir * 0.5);
            show();
        }
        drive(0, 0, 0);
    }

    private double avgForwardTicks() {
        return (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4.0;
    }

    private double avgStrafeTicks() {   // + when sliding right (matches the wheel math in drive())
        return (fl.getCurrentPosition() - bl.getCurrentPosition() - fr.getCurrentPosition() + br.getCurrentPosition()) / 4.0;
    }

    /** Look for a ball for a moment without moving. Returns the biggest one seen, or null. */
    private HuskyLens.Block lookFor(double seconds) {
        drive(0, 0, 0);
        seenFrames = 0;
        HuskyLens.Block best = null;
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            HuskyLens.Block b = findPollen();
            if (b != null && (best == null || b.width > best.width)) best = b;
            show();
        }
        return best;
    }

    private void waitSeconds(double s) {
        drive(0, 0, 0);
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < s) show();
    }

    /**
     * The pollen ball to chase, after it's been seen CONFIRM_FRAMES in a row.
     * Not locked -> the biggest (closest) box. Locked -> only the box closest to where the locked ball was,
     * so it doesn't switch to a different ball halfway there.
     */
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
                if (lockedBall == null) {
                    if (best == null || b.width > best.width) best = b;
                } else {
                    double jump = Math.hypot(b.x - lockedBall.x, b.y - lockedBall.y);
                    if (jump > LOCK_JUMP_PX) continue;          // a different ball - ignore it
                    if (best == null || jump < Math.hypot(best.x - lockedBall.x, best.y - lockedBall.y)) best = b;
                }
            }
        }
        if (best == null) { seenFrames = 0; return null; }
        lastBall = best;
        seenFrames++;
        return seenFrames >= CONFIRM_FRAMES ? best : null;
    }

    /** forward (+ = ahead), turn (+ = left), right (+ = slide right). Same mecanum math as the collector, times SPEED. */
    private void drive(double forward, double turn, double right) {
        lastFwd = forward; lastTurn = turn; lastRight = right;
        double r = -turn;
        double lf = forward + right + r, lb = forward - right + r, rf = forward - right - r, rb = forward + right - r;
        double biggest = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb))));
        double k = SPEED / biggest;
        fl.setPower(lf * k); bl.setPower(lb * k); fr.setPower(rf * k); br.setPower(rb * k);
    }

    private void intake(boolean on) {
        double p = on ? 1 : 0;
        intakemotor.setPower(p); leftintake.setPower(p); rightintake.setPower(p);
    }

    private void show() {
        telemetry.addData("Status", status);
        telemetry.addData("Locked on", lockedBall != null ? "yes" : "no");
        telemetry.addData("Collected", "%d   (missed %d, retries %d)", collected, missed, retries);
        if (lastBall != null) {
            telemetry.addData("Ball box", "x %d  y %d  w %d   (close: w>=%d or y>=%d)",
                    lastBall.x, lastBall.y, lastBall.width, INTAKE_ZONE_WIDTH, INTAKE_ZONE_Y);
        }
        telemetry.addData("Drive", "forward %.2f  turn left %.2f  slide right %.2f", lastFwd, lastTurn, lastRight);
        telemetry.addData("Wheels moved", "%.1f in", avgForwardTicks() * INCHES_PER_TICK);
        telemetry.update();
    }
}

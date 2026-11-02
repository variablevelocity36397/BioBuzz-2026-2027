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
  POLLEN FOLLOWER - find one ball, lock on, drive to it, intake it, repeat
  ------------------------------------------------------------------------
  The robot goes through these steps over and over:
    1. SEARCH : no ball in view -> SPIN in place (toward the side it last saw a ball) until it finds one
    2. CHASE  : LOCK ON to ONE ball (it keeps following that same ball even if others are in view),
                turn to keep it centered and drive toward it.
                When the ball is close (in the INTAKE ZONE) the intake turns ON.
    3. GRAB   : ball is right at the intake (or just went under the camera) ->
                intake ON and creep forward for INTAKE_TIME seconds, then go back to SEARCH

  Uses the same motor/intake names and directions as "Pollen Collector (P)".
  If it turns the WRONG way, flip TURN_SIGN. Drives the wrong way: FORWARD_SIGN.
  Spins the wrong way while searching: flip SEARCH_SIGN.
*/
@BulkCaching   // Marrow: faster hardware reads
@Autonomous(name = "Pollen Follower", group = "Pollen")
public class PollenFollower extends LinearOpMode {

    // ---------- Speed ----------
    private static final double SPEED = 0.5;              // multiplies every drive power (1.0 = full speed)

    // ---------- Searching (no ball in view) ----------
    private static final double SEARCH_TURN  = 0.35;      // spin power while looking for a ball
    private static final double SEARCH_SIGN  = 1.0;       // set to -1.0 if it spins the opposite way you expect
    private static final double LOST_GRACE   = 0.4;       // seconds to keep going after losing the ball before searching

    // ---------- Turning toward the ball ----------
    private static final double KP_TURN      = 0.005;     // turn power per PIXEL the ball is off-center
    private static final double MAX_TURN     = 0.5;
    private static final int    CENTER_DEADBAND = 10;     // pixels: this close to center counts as centered
    private static final double TURN_SIGN    = 1.0;       // set to -1.0 if it turns AWAY from the ball

    // ---------- Driving to the ball ----------
    private static final double APPROACH_POWER = 0.45;    // forward power when the ball is far
    private static final double MIN_APPROACH   = 0.25;    // forward power when the ball is close
    private static final double FORWARD_SIGN   = 1.0;     // set to -1.0 if it backs away from the ball

    // ---------- Intake ----------
    // A ball is "in the intake zone" (intake turns ON) if EITHER is true (watch "Ball box" in telemetry to tune):
    private static final int    INTAKE_ZONE_WIDTH = 45;   // its box is at least this many pixels wide, OR
    private static final int    INTAKE_ZONE_Y     = 170;  // its center is this low on the screen (0 top .. 240 bottom)
    private static final int    CLOSE_WIDTH       = 90;   // box this wide = ball is right at the intake -> GRAB
    private static final double GRAB_FORWARD      = 0.3;  // forward power while driving over the ball
    private static final double INTAKE_TIME       = 1.5;  // seconds to intake + creep forward
    private static final double INTAKE_SETTLE     = 0.3;  // extra seconds of intake (standing still) to pull it in

    // ---------- Locking onto ONE ball ----------
    private static final int    LOCK_JUMP_PX = 70;        // a box further than this from the locked ball = a DIFFERENT ball

    // ---------- Camera ----------
    private static final int    POLLEN_ID = 1;            // HuskyLens learned ID for pollen
    private static final int    SCREEN_W  = 320;
    private static final int    SCREEN_H  = 240;

    // Filters (only follow things that look like pollen)
    private static final int    MIN_BOX_PX     = 10;
    private static final double MIN_ASPECT     = 0.6, MAX_ASPECT = 1.6;
    private static final int    MIN_Y          = 60;      // ignore boxes near the top of the screen
    private static final int    CONFIRM_FRAMES = 2;       // must see it this many frames in a row

    private DcMotor fl, fr, bl, br, intakemotor;
    private CRServo leftintake, rightintake;
    private HuskyLens huskyLens;

    private enum State { SEARCH, CHASE, GRAB }
    private State state = State.SEARCH;

    private int seenFrames = 0;
    private HuskyLens.Block target = null;      // the ONE ball we're locked onto (null = none)
    private int lastSide = 1;                   // +1 = ball was last seen on the left, -1 = right (which way to spin)
    private boolean wasClose = false;           // was the locked ball in the intake zone last time we saw it?
    private final ElapsedTime lostTimer = new ElapsedTime();
    private int collected = 0;
    private String status = "Waiting";
    private double lastForward = 0, lastTurn = 0;

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
        telemetry.addLine("Press START - it will spin to find pollen, drive to it and intake it.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block ball = findTarget();

            switch (state) {
                case SEARCH:
                    if (ball != null) {
                        state = State.CHASE;
                    } else {
                        status = "Searching - spinning " + (lastSide > 0 ? "left" : "right");
                        intakeOff();
                        drive(0, SEARCH_TURN * lastSide * SEARCH_SIGN);
                    }
                    break;

                case CHASE:
                    if (ball == null) {
                        if (wasClose) {
                            // Ball went under the camera right in front of the intake - grab it
                            state = State.GRAB;
                        } else if (lostTimer.seconds() > LOST_GRACE) {
                            target = null;
                            state = State.SEARCH;
                        } else {
                            status = "Lost ball - waiting a moment";
                            drive(0, 0);
                        }
                        break;
                    }
                    lostTimer.reset();
                    if (ball.width >= CLOSE_WIDTH) {
                        state = State.GRAB;
                        break;
                    }
                    chase(ball);
                    break;

                case GRAB:
                    grab();
                    state = State.SEARCH;
                    break;
            }
            show();
        }
        drive(0, 0);
        intakeOff();
    }

    /** Turn to center the locked ball and drive at it. Intake turns on once it's in the intake zone. */
    private void chase(HuskyLens.Block ball) {
        int xErr = ball.x - SCREEN_W / 2;
        lastSide = xErr <= 0 ? 1 : -1;     // remember which side it was on, so a search spins that way

        // Ball right of center -> turn right (negative = clockwise). Left of center -> turn left.
        double turn = Math.abs(xErr) <= CENTER_DEADBAND ? 0
                : Range.clip(-KP_TURN * xErr, -MAX_TURN, MAX_TURN) * TURN_SIGN;

        boolean inIntakeZone = ball.width >= INTAKE_ZONE_WIDTH || ball.y >= INTAKE_ZONE_Y;
        wasClose = inIntakeZone;

        double forward;
        if (inIntakeZone) {
            status = "IN INTAKE ZONE - intake on, driving onto it";
            intakeOn();
            forward = GRAB_FORWARD;
        } else {
            status = "Locked on - driving to ball";
            intakeOff();
            // slow down as the ball gets bigger (closer)
            double closeness = Range.clip((double) ball.width / INTAKE_ZONE_WIDTH, 0, 1);
            forward = APPROACH_POWER - (APPROACH_POWER - MIN_APPROACH) * closeness;
        }
        // don't charge forward while the ball is way off to the side - line up first
        forward *= 1.0 - Range.clip(Math.abs(xErr) / (SCREEN_W / 2.0), 0, 1);
        drive(forward * FORWARD_SIGN, turn);
    }

    /** Intake ON and creep forward over the ball, then let the intake finish pulling it in. */
    private void grab() {
        status = "GRABBING - intake on";
        intakeOn();
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < INTAKE_TIME) {
            drive(GRAB_FORWARD * FORWARD_SIGN, 0);
            show();
        }
        drive(0, 0);
        t.reset();
        while (opModeIsActive() && t.seconds() < INTAKE_SETTLE) idle();
        intakeOff();
        collected++;
        target = null;
        wasClose = false;
        seenFrames = 0;
    }

    /**
     * Returns the ONE ball we're locked onto.
     * No lock yet -> pick the biggest (closest) pollen box. Locked -> only accept the box closest to where
     * the locked ball was, so it doesn't switch to a different ball halfway there.
     */
    private HuskyLens.Block findTarget() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        HuskyLens.Block best = null;
        double bestScore = Double.MAX_VALUE;
        if (blocks != null) {
            for (HuskyLens.Block b : blocks) {
                if (b.id != POLLEN_ID) continue;
                if (b.width < MIN_BOX_PX || b.height < MIN_BOX_PX) continue;
                double aspect = (double) b.width / b.height;
                if (aspect < MIN_ASPECT || aspect > MAX_ASPECT) continue;
                if (b.y < MIN_Y) continue;

                double score;
                if (target == null) {
                    score = -b.width;                                    // biggest box = closest ball
                } else {
                    double jump = Math.hypot(b.x - target.x, b.y - target.y);
                    if (jump > LOCK_JUMP_PX) continue;                   // a different ball - ignore it
                    score = jump;
                }
                if (score < bestScore) { bestScore = score; best = b; }
            }
        }
        if (best == null) {
            seenFrames = 0;
            if (state == State.SEARCH) target = null;   // not locked yet - any ball can be picked next time
            return null;
        }
        target = best;
        seenFrames++;
        return seenFrames >= CONFIRM_FRAMES ? best : null;
    }

    /** forward (+ = ahead), turn (+ = turn left). Same mecanum math as the collector, times SPEED. */
    private void drive(double forward, double turn) {
        lastForward = forward;
        lastTurn = turn;
        double r = -turn;
        double lf = forward + r, lb = forward + r, rf = forward - r, rb = forward - r;
        double biggest = Math.max(1.0, Math.max(Math.abs(lf), Math.abs(rf)));
        double k = SPEED / biggest;
        fl.setPower(lf * k); bl.setPower(lb * k); fr.setPower(rf * k); br.setPower(rb * k);
    }

    private void intakeOn()  { intakemotor.setPower(1); leftintake.setPower(1); rightintake.setPower(1); }
    private void intakeOff() { intakemotor.setPower(0); leftintake.setPower(0); rightintake.setPower(0); }

    private void show() {
        telemetry.addData("State", state);
        telemetry.addData("Status", status);
        telemetry.addData("Collected", collected);
        if (target != null) {
            telemetry.addData("Ball box", "x %d (center %d)  y %d  w %d  (intake zone: w>=%d or y>=%d, grab at w>=%d)",
                    target.x, SCREEN_W / 2, target.y, target.width, INTAKE_ZONE_WIDTH, INTAKE_ZONE_Y, CLOSE_WIDTH);
        } else {
            telemetry.addData("Ball box", "none (screen %dx%d)", SCREEN_W, SCREEN_H);
        }
        telemetry.addData("Drive", "forward %.2f   turn left %.2f", lastForward, lastTurn);
        telemetry.update();
    }
}

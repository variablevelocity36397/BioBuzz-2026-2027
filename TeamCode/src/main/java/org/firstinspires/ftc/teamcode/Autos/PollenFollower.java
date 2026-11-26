package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.skeletonarmy.marrow.BulkCaching;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PathRoute;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.Point;

import java.util.ArrayList;
import java.util.List;

/*
  POLLEN FOLLOWER - follow the pollen and INTAKE it
  -------------------------------------------------
  PATH = true (default, Nov 24): when it sees a ball it works out WHERE the ball is (distance from the box
  size, angle from the box position), makes a path to that spot (plus PATH_OVERSHOOT_IN past it) and DRIVES
  that path fast, tracking its own position with the wheel encoders + Control Hub IMU. The camera keeps
  fixing the path while it can see the ball; when the ball goes under the camera it finishes the path blind
  with the intake on. Then it searches for the next ball. No limit on balls - runs until STOP.
  Tune: DIST_K (look at "Ball est" telemetry vs a tape measure), CAMERA_FOV_DEG, PATH_POWER, K_STATIC.

  PATH = false = the older camera-chasing modes below:
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

    private static final boolean COLLECT = true;          // true = chase + INTAKE, false = chase only
    // SWEEP = go to the ball, intake it, roll a few inches forward, then look for the next ball
    // (no backing up to check). false = the careful way (pass, back up, check, retry).
    private static final boolean SWEEP = true;

    // ---------- PATH mode: plan a path to the ball and drive it ----------
    private static final boolean PATH = true;
    private static final double PATH_POWER       = 0.9;   // forward power on the path (fast)
    private static final double PICKUP_POWER     = 0.55;  // max forward power once the intake is on (so it doesn't bounce off)
    private static final double PATH_MIN_POWER   = 0.25;  // power right at the end of the path
    private static final double PATH_SLOW_PER_IN = 0.04;  // slow down near the end: power limit = MIN + this * inches left
    private static final double KP_HEADING       = 1.8;   // turn power per RADIAN the path is off to the side
    private static final double PATH_MAX_TURN    = 0.6;
    private static final double PATH_OVERSHOOT_IN = 6.0;  // drive the robot's CENTER this far PAST the ball so it's surely in
    private static final double INTAKE_ON_IN     = 18.0;  // turn the intake on when the ball is this close (robot center)
    private static final double AFTER_INTAKE_EXTRA_IN = 6.0;  // once the intake turns on, make the path this much LONGER (go a bit further)
    private static final double TURN_IN_PLACE_DEG = 20.0; // ball more than this far to the side -> stop going forward and spin with ALL 4 wheels
    private static final double PATH_DONE_IN     = 1.0;   // path finished when this close to the end
    private static final double PATH_TIMEOUT     = 4.0;   // safety: give up on one path after this many seconds
    private static final double REPLAN_EVERY     = 0.25;  // seconds between new Weaver paths while balls are in view
    private static final double LOOKAHEAD_IN     = 8.0;   // aim at the path this far ahead of the robot (bigger = smoother, cuts corners)
    private static final int    SAMPLES_PER_SEGMENT = 20; // points per Weaver curve
    // Weaver robot size (inches). WEAVER_INTAKE_WIDTH = HALF the real intake width (Marrow docs say so)
    private static final double ROBOT_LENGTH_IN = 18.0, ROBOT_WIDTH_IN = 18.0, WEAVER_INTAKE_WIDTH = 4.0;
    // Where is the ball? distance (in) from camera ~= DIST_K / box width (px). Measure: put a ball 12 in in front
    // of the camera, read the box width w, set DIST_K = 12 * w.
    private static final double DIST_K           = 750.0;
    private static final double CAMERA_FOV_DEG  = 60.0;   // HuskyLens horizontal view angle
    private static final double CAMERA_FORWARD_IN = 7.0;  // camera is this far in front of the robot's center
    // Heading: Control Hub IMU ("imu"). If there's no IMU it uses the wheels (needs TRACK_WIDTH_IN).
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection  HUB_USB  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    private static final double TRACK_WIDTH_IN   = 15.0;

    // ---------- Static friction ----------
    // Small powers don't move the robot at all (static friction). Any wheel that's supposed to move gets at least this.
    private static final double K_STATIC = 0.1;
    private static final double SWEEP_PASS_IN    = 6.0;   // after a ball vanishes under the camera, keep going this far
    private static final double SWEEP_POWER      = 0.5;   // forward power for that pass
    private static final double SWEEP_INTAKE_HOLD = 0.0;  // 0 = intake OFF while finding the next ball (raise to keep it on longer)

    // ---------- Speed ----------
    private static final double SPEED = 1.0;              // multiplies every drive power (1.0 = full speed)

    // ---------- Smoothness (less jerky) ----------
    private static final double ACCEL_RATE = 3.5;         // how fast power can go UP, per second (0 -> 0.9 in ~0.25 s). Lower = smoother
    private static final double DECEL_RATE = 5.0;         // how fast power can go DOWN, per second
    private static final double X_SMOOTH   = 0.5;         // 0..1: how much each new camera frame counts for steering. Lower = smoother turning

    // ---------- Searching (no ball in view) ----------
    private static final double SEARCH_TURN  = 0.3;       // spin power while looking for a ball
    private static final double SEARCH_SIGN  = 1.0;       // -1.0 if it spins the opposite way you expect
    private static final double LOST_GRACE   = 0.4;       // seconds to wait after losing the ball before spinning
    private static final double FOUND_STOP   = 0.0;       // seconds to STOP when it first spots a ball (0 = don't stop, just turn toward it smoothly)

    // ---------- Locking onto ONE ball ----------
    private static final int    LOCK_JUMP_PX = 70;        // a box further than this from the locked ball = a DIFFERENT ball

    // ---------- How it follows left/right movement ----------
    private enum Track { TURN, STRAFE, BOTH }
    private static final Track TRACK_MODE = Track.TURN;   // TURN = rotate to face the ball, STRAFE = slide, BOTH
    private static final double KP_TURN      = 0.005;     // turn power per PIXEL the ball is off-center
    private static final double MAX_TURN     = 0.45;      // fastest it will turn while following
    private static final double TURN_SIGN    = 1.0;       // -1.0 if it turns AWAY from the ball
    private static final double KP_STRAFE    = 0.008;
    private static final double MAX_STRAFE   = 0.8;
    private static final double STRAFE_SIGN  = 1.0;
    private static final int    CENTER_DEADBAND = 12;     // pixels: this close to center counts as centered

    // ---------- Driving toward the ball ----------
    private static final double APPROACH_POWER = 0.8;     // forward power when the ball is far (was 0.65)
    private static final double MIN_APPROACH   = 0.45;    // forward power when it is getting close (was 0.35)
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
    private static final double LOOK_TIME   = 0.3;        // seconds to look after backing up
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

    private DcMotor fl, fr, bl, br;
    private RobotNames.Intake intakeParts;
    private HuskyLens huskyLens;

    private int seenFrames = 0, collected = 0, missed = 0, retries = 0;
    private boolean wasClose = false;
    private int retrySide = 1;
    private String status = "Waiting for pollen";
    private HuskyLens.Block lastBall = null;
    private HuskyLens.Block lockedBall = null;  // the ONE ball we're chasing (null = not locked)
    private int lastSide = 1;                   // +1 = ball was last seen on the left, -1 = right (which way to spin)
    private final ElapsedTime lostTimer = new ElapsedTime();
    private final ElapsedTime sweepTimer = new ElapsedTime();   // time since the last ball went in (SWEEP)
    private boolean sweptOnce = false;
    private double lastFwd = 0, lastTurn = 0, lastRight = 0;     // what we ASKED for
    private double curFwd = 0, curTurn = 0, curRight = 0;        // what the wheels are actually getting (ramped)
    private final ElapsedTime rampTimer = new ElapsedTime();
    private double filtX = SCREEN_W / 2.0;                       // smoothed ball x (steering)

    // PATH mode state. Position is in inches from where the robot started: x = forward, y = left, h = heading (radians, + = left)
    private IMU imu;
    private double poseX = 0, poseY = 0, poseH = 0, lastFTicks = 0, lastSTicks = 0;
    private boolean hasPath = false, intakeLatched = false;
    private double ballEstIn = 0;
    private List<Point> pathPts = new ArrayList<>();   // the Weaver path, as points
    private List<Point> targets = new ArrayList<>();   // balls this path visits
    private int progress = 0;                          // index of the path point we are at
    private String weaverStatus = "no path yet";
    private final ElapsedTime replanTimer = new ElapsedTime();
    private final ElapsedTime pathTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Names come from RobotNames.java (works with "BioBuzz Robot" and the old "Starter Bot")
        fl = RobotNames.need(hardwareMap, DcMotor.class, RobotNames.FL, "front left wheel");
        fr = RobotNames.need(hardwareMap, DcMotor.class, RobotNames.FR, "front right wheel");
        bl = RobotNames.need(hardwareMap, DcMotor.class, RobotNames.BL, "back left wheel");
        br = RobotNames.need(hardwareMap, DcMotor.class, RobotNames.BR, "back right wheel");
        fl.setDirection(PollenCollector.FL_DIR);
        fr.setDirection(PollenCollector.FR_DIR);
        bl.setDirection(PollenCollector.BL_DIR);
        br.setDirection(PollenCollector.BR_DIR);
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // we set power ourselves, but still read ticks
        }

        intakeParts = new RobotNames.Intake(hardwareMap);   // every intake part in the config

        huskyLens = RobotNames.need(hardwareMap, HuskyLens.class, RobotNames.HUSKY, "HuskyLens");
        boolean found = huskyLens.knock();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        }
        {
            // Marrow Weaver: robot size + intake. robotWidth = along the forward axis, robotHeight = across (Marrow names)
            Weaver.setConfig(new PathConfig()
                    .intakeWidth(WEAVER_INTAKE_WIDTH)
                    .robotWidth(ROBOT_LENGTH_IN)
                    .robotHeight(ROBOT_WIDTH_IN));
        }

        telemetry.addData("HuskyLens", found ? "ready" : "NOT FOUND - check wiring / config name 'husky lens'");
        telemetry.addData("Intake parts", intakeParts.describe());
        telemetry.addData("Heading from", imu != null ? "IMU" : "wheels (no 'imu' in config)");
        telemetry.addData("Mode", !COLLECT ? "follow only" : PATH ? "Marrow WEAVER path through the balls + INTAKE" : "follow + INTAKE");
        telemetry.update();
        waitForStart();
        if (imu != null) imu.resetYaw();
        lastFTicks = avgForwardTicks();
        lastSTicks = avgStrafeTicks();

        while (opModeIsActive()) {
            if (COLLECT && PATH) { pathStep(); continue; }
            HuskyLens.Block ball = findPollen();

            if (ball == null) {
                if (COLLECT && wasClose && SWEEP) {
                    // SWEEP: drive through it with the intake on, then go straight for the next ball
                    wasClose = false;
                    sweepThrough();
                } else if (COLLECT && wasClose) {
                    // It slid under the camera right in front of the intake -> finish the pickup
                    wasClose = false;
                    pickUpAndCheck();
                } else if (lockedBall != null && lostTimer.seconds() < LOST_GRACE) {
                    // Camera blinked - keep doing what we were doing instead of slamming to a stop
                    status = "Lost the ball for a moment - keep going";
                    intake(sweeping());
                    drive(lastFwd, lastTurn, lastRight);
                } else {
                    // Nothing in view -> spin toward where we last saw a ball
                    lockedBall = null;
                    status = "Searching - spinning " + (lastSide > 0 ? "left" : "right");
                    intake(sweeping());
                    drive(0, SEARCH_TURN * lastSide * SEARCH_SIGN, 0);
                }
                show();
                continue;
            }

            if (lockedBall == null) filtX = ball.x;   // new ball -> start steering from where it really is
            if (lockedBall == null && !sweeping() && FOUND_STOP > 0) {
                // Just spotted a ball after searching -> STOP first so the spin doesn't carry it past
                lockedBall = ball;
                status = "Found a ball - stopping";
                waitSeconds(FOUND_STOP);
                lostTimer.reset();
                continue;
            }
            lockedBall = ball;                    // stay on THIS ball
            lostTimer.reset();
            filtX += X_SMOOTH * (ball.x - filtX); // smooth out camera wobble
            int xErr = (int) Math.round(filtX - SCREEN_W / 2.0);
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
                intake(sweeping());   // intake only turns on when the ball is close
                double closeness = Range.clip((double) ball.width / INTAKE_ZONE_WIDTH, 0, 1);
                double fwd = MIN_APPROACH + (APPROACH_POWER - MIN_APPROACH) * (1 - closeness);
                fwd *= 1.0 - 0.7 * Range.clip(Math.abs(xErr) / (SCREEN_W / 2.0), 0, 1);  // line up first
                drive(fwd * FORWARD_SIGN, side[0], side[1]);
            }
            show();
        }
        stopNow();
        intake(false);
    }

    // ======================= PATH mode (Marrow WEAVER) =======================
    // Every pollen ball the camera sees becomes a Weaver TARGET. Weaver picks the best order and makes ONE smooth
    // path through all of them (robot center over each ball, facing along the path so the intake sweeps it up).
    // We drive that path ourselves (pure pursuit: aim at a point LOOKAHEAD_IN ahead on the path), using our
    // encoder + IMU position. Pedro's follower isn't used because Pedro isn't tuned on this robot yet.

    /** One loop of PATH mode: track position, plan/fix the path from the camera, drive it, pick up. */
    private void pathStep() {
        updatePose();
        List<Point> balls = seeBalls();
        boolean closeOnCamera = false;
        if (!intakeLatched && !balls.isEmpty() && (!hasPath || replanTimer.seconds() > REPLAN_EVERY)) {
            planWithWeaver(balls);   // once the intake is on, finish the path blind (camera can't see under the robot)
        }
        if (lastBall != null && seenFrames > 0)
            closeOnCamera = lastBall.width >= INTAKE_ZONE_WIDTH || lastBall.y >= INTAKE_ZONE_Y;

        if (!hasPath) {
            intake(false);
            status = "Searching - spinning " + (lastSide > 0 ? "left" : "right");
            drive(0, SEARCH_TURN * lastSide * SEARCH_SIGN, 0);
            show();
            return;
        }

        // Where are we on the path? (closest point, only searching forward so we never go backwards)
        int last = pathPts.size() - 1;
        double best = Double.MAX_VALUE;
        for (int i = progress; i <= Math.min(last, progress + 30); i++) {
            double d = dist(pathPts.get(i));
            if (d < best) { best = d; progress = i; }
        }
        Point end = pathPts.get(last);
        double ex = end.getX() - poseX, ey = end.getY() - poseY;
        boolean passed = ex * Math.cos(poseH) + ey * Math.sin(poseH) < 0 && progress >= last - 1;
        double remaining = remainingLength();
        if ((progress >= last && dist(end) < PATH_DONE_IN) || passed || pathTimer.seconds() > PATH_TIMEOUT) {
            finishPickup();
            return;
        }

        // Intake on when we get near a ball, then keep it on for the rest of the path
        if (!intakeLatched && (closeOnCamera || nearestTargetDist() < INTAKE_ON_IN)) {
            intakeLatched = true;
            extendEnd(AFTER_INTAKE_EXTRA_IN);   // go a bit further once the intake starts
            last = pathPts.size() - 1;
            remaining = remainingLength();
        }
        intake(intakeLatched);

        // Aim at a point a little way ahead on the path
        Point aim = pathPts.get(last);
        for (int i = progress; i <= last; i++) {
            if (dist(pathPts.get(i)) >= LOOKAHEAD_IN) { aim = pathPts.get(i); break; }
        }
        double err = wrap(Math.atan2(aim.getY() - poseY, aim.getX() - poseX) - poseH);   // + = aim point is to the left

        double turn = Range.clip(KP_HEADING * err, -PATH_MAX_TURN, PATH_MAX_TURN) * TURN_SIGN;
        // Far off to the side -> turn IN PLACE (all 4 wheels: left side back, right side forward) before driving
        // (not in the last 8 in - there it just keeps rolling so it doesn't spin around next to the ball)
        double fwd = Math.abs(err) > Math.toRadians(TURN_IN_PLACE_DEG) && remaining > 8.0 ? 0
                   : PATH_POWER * Math.cos(err);
        fwd = Math.min(fwd, PATH_MIN_POWER + PATH_SLOW_PER_IN * remaining);  // slow down near the end
        if (intakeLatched) fwd = Math.min(fwd, PICKUP_POWER);
        status = (intakeLatched ? "Weaver path - intake ON" : "Weaver path to " + targets.size() + " ball(s)");
        drive(fwd * FORWARD_SIGN, turn, 0);
        show();
    }

    /** All pollen balls in view right now, as field points (inches, same frame as poseX/poseY). */
    private List<Point> seeBalls() {
        List<Point> out = new ArrayList<>();
        HuskyLens.Block[] blocks = huskyLens.blocks();
        HuskyLens.Block biggest = null;
        if (blocks != null) {
            for (HuskyLens.Block b : blocks) {
                if (b.id != POLLEN_ID) continue;
                if (b.width < MIN_BOX_PX || b.height < MIN_BOX_PX) continue;
                double aspect = (double) b.width / b.height;
                if (aspect < MIN_ASPECT || aspect > MAX_ASPECT) continue;
                if (b.y < MIN_Y) continue;
                out.add(ballToField(b));
                if (biggest == null || b.width > biggest.width) biggest = b;
            }
        }
        if (biggest == null) { seenFrames = 0; return out; }
        lastBall = biggest;
        lastSide = biggest.x <= SCREEN_W / 2 ? 1 : -1;
        seenFrames++;
        if (seenFrames < CONFIRM_FRAMES && !hasPath) out.clear();   // must see balls a couple of frames before trusting them
        return out;
    }

    /** Camera box -> where that ball is on the field. */
    private Point ballToField(HuskyLens.Block b) {
        double ang = -(b.x - SCREEN_W / 2.0) / (SCREEN_W / 2.0) * Math.toRadians(CAMERA_FOV_DEG / 2.0);
        double d = DIST_K / Math.max(b.width, 1);
        if (b == lastBall || lastBall == null || b.width >= lastBall.width) ballEstIn = d;
        double camX = poseX + CAMERA_FORWARD_IN * Math.cos(poseH), camY = poseY + CAMERA_FORWARD_IN * Math.sin(poseH);
        return new Point(camX + d * Math.cos(poseH + ang), camY + d * Math.sin(poseH + ang));
    }

    /** Ask Weaver for a path from where we are through the balls, then add the overshoot at the end. */
    private void planWithWeaver(List<Point> balls) {
        Point start = new Point(poseX, poseY, poseH);
        List<Point> newPts = new ArrayList<>();
        int visited;
        try {
            PathResult result = Weaver.builder()
                    .start(start)
                    .targets(balls)
                    .build();
            PathRoute route = result.getPath();
            newPts.addAll(route.sample(SAMPLES_PER_SEGMENT));
            visited = result.getVisitedTargets().size();
            targets = new ArrayList<>(result.getVisitedTargets());
            double endHeading = route.getHeading(route.getSegmentCount());
            Point last = newPts.get(newPts.size() - 1);
            newPts.add(new Point(last.getX() + PATH_OVERSHOOT_IN * Math.cos(endHeading),
                                 last.getY() + PATH_OVERSHOOT_IN * Math.sin(endHeading)));
            weaverStatus = "ok - " + route.getSegmentCount() + " curves, " + visited + " ball(s)";
        } catch (RuntimeException e) {
            // Weaver couldn't make a path -> straight line to the closest ball
            Point closest = balls.get(0);
            for (Point p : balls) if (dist(p) < dist(closest)) closest = p;
            double h = Math.atan2(closest.getY() - poseY, closest.getX() - poseX);
            newPts.add(start);
            newPts.add(closest);
            newPts.add(new Point(closest.getX() + PATH_OVERSHOOT_IN * Math.cos(h), closest.getY() + PATH_OVERSHOOT_IN * Math.sin(h)));
            targets = new ArrayList<>();
            targets.add(closest);
            weaverStatus = "failed (" + e.getMessage() + ") - straight line";
        }
        if (newPts.size() < 2) return;
        pathPts = newPts;
        progress = 0;
        if (!hasPath) {
            hasPath = true;
            intakeLatched = false;
            pathTimer.reset();
        }
        replanTimer.reset();
    }

    /** Make the path longer in the direction it's heading at the end. */
    private void extendEnd(double inches) {
        int n = pathPts.size();
        Point a = pathPts.get(n - 2), b = pathPts.get(n - 1);
        double h = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
        pathPts.add(new Point(b.getX() + inches * Math.cos(h), b.getY() + inches * Math.sin(h)));
    }

    private double remainingLength() {
        double len = dist(pathPts.get(progress));
        for (int i = progress; i < pathPts.size() - 1; i++) len += pathPts.get(i).distanceTo(pathPts.get(i + 1));
        return len;
    }

    private double nearestTargetDist() {
        double best = Double.MAX_VALUE;
        for (Point t : targets) best = Math.min(best, dist(t));
        return best;
    }

    private double dist(Point p) {
        return Math.hypot(p.getX() - poseX, p.getY() - poseY);
    }

    /** End of the path: stop with the intake still on so the balls settle, then go look for more. */
    private void finishPickup() {
        status = "End of path - finishing the pickup";
        intake(true);
        waitSeconds(INTAKE_EXTRA_TIME);
        intake(false);
        collected += Math.max(1, targets.size());
        hasPath = false;
        intakeLatched = false;
        targets = new ArrayList<>();
        seenFrames = 0;
        status = "Got them (" + collected + ") - looking for more";
    }

    /** Track where the robot is: wheel encoders for distance, IMU (or wheels) for heading. */
    private void updatePose() {
        double f = avgForwardTicks(), s = avgStrafeTicks();
        double dF = (f - lastFTicks) * INCHES_PER_TICK, dR = (s - lastSTicks) * INCHES_PER_TICK;
        lastFTicks = f; lastSTicks = s;
        if (imu != null) {
            poseH = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        } else {
            double right = (fr.getCurrentPosition() + br.getCurrentPosition()) / 2.0;
            double left = (fl.getCurrentPosition() + bl.getCurrentPosition()) / 2.0;
            poseH = (right - left) * INCHES_PER_TICK / TRACK_WIDTH_IN;
        }
        poseX += dF * Math.cos(poseH) + dR * Math.sin(poseH);
        poseY += dF * Math.sin(poseH) - dR * Math.cos(poseH);
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    /** SWEEP: intake on, roll a few inches forward past the ball, let it settle, then go find the next one. */
    private void sweepThrough() {
        lockedBall = null;                      // next loop searches / locks onto the next ball
        status = "Intaking - rolling forward a few inches";
        intake(true);
        driveDistance(SWEEP_PASS_IN, SWEEP_POWER, PASS_TIMEOUT);
        waitSeconds(INTAKE_EXTRA_TIME);         // stopped, intake still on so the ball settles in
        intake(false);
        collected++;
        status = "Got it (" + collected + ") - looking for the next ball";
        sweptOnce = true;
        sweepTimer.reset();
    }

    /** True while the intake should stay on between balls in SWEEP mode. */
    private boolean sweeping() {
        return COLLECT && SWEEP && sweptOnce && sweepTimer.seconds() < SWEEP_INTAKE_HOLD;
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
        stopNow();
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
        stopNow();
    }

    private double avgForwardTicks() {
        return (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4.0;
    }

    private double avgStrafeTicks() {   // + when sliding right (matches the wheel math in drive())
        return (fl.getCurrentPosition() - bl.getCurrentPosition() - fr.getCurrentPosition() + br.getCurrentPosition()) / 4.0;
    }

    /** Look for a ball for a moment without moving. Returns the biggest one seen, or null. */
    private HuskyLens.Block lookFor(double seconds) {
        stopNow();
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
        stopNow();
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
        // Need CONFIRM_FRAMES in a row to START chasing a ball, but once locked, one frame is enough
        // (so a single blink doesn't make it stop and start again)
        return (seenFrames >= CONFIRM_FRAMES || lockedBall != null) ? best : null;
    }

    /** forward (+ = ahead), turn (+ = left), right (+ = slide right). Same mecanum math as the collector, times SPEED. */
    private void drive(double forward, double turn, double right) {
        lastFwd = forward; lastTurn = turn; lastRight = right;
        // RAMP: don't jump straight to the new power, move toward it a little each loop (smooth, not jerky)
        double dt = Math.min(rampTimer.seconds(), 0.1);
        rampTimer.reset();
        curFwd = ramp(curFwd, forward, dt);
        curTurn = ramp(curTurn, turn, dt);
        curRight = ramp(curRight, right, dt);
        setWheels(curFwd, curTurn, curRight);
    }

    private static double ramp(double cur, double target, double dt) {
        boolean slowingDown = Math.abs(target) < Math.abs(cur) || cur * target < 0;
        double step = (slowingDown ? DECEL_RATE : ACCEL_RATE) * dt;
        return cur + Range.clip(target - cur, -step, step);
    }

    /** Stop right now (no ramp) - used for planned stops like after a pickup. */
    private void stopNow() {
        lastFwd = lastTurn = lastRight = 0;
        curFwd = curTurn = curRight = 0;
        setWheels(0, 0, 0);
        rampTimer.reset();
    }

    private void setWheels(double forward, double turn, double right) {
        double r = -turn;
        double lf = forward + right + r, lb = forward - right + r, rf = forward - right - r, rb = forward + right - r;
        double biggest = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb))));
        double k = SPEED / biggest;
        fl.setPower(beatFriction(lf * k)); bl.setPower(beatFriction(lb * k));
        fr.setPower(beatFriction(rf * k)); br.setPower(beatFriction(rb * k));
    }

    /** Static friction: a wheel that should move gets at least K_STATIC power, so small powers still move it. */
    private static double beatFriction(double p) {
        if (Math.abs(p) < 0.02) return 0;
        return Math.signum(p) * (K_STATIC + (1 - K_STATIC) * Math.abs(p));
    }

    private void intake(boolean on) {
        double p = on ? 1 : 0;
        intakeParts.setPower(p);
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
        if (PATH) {
            telemetry.addData("Robot at", "x %.1f  y %.1f in   heading %.0f deg", poseX, poseY, Math.toDegrees(poseH));
            telemetry.addData("Ball est", "%.1f in from camera (DIST_K / box width)", ballEstIn);
            telemetry.addData("Weaver", weaverStatus);
            if (hasPath && !pathPts.isEmpty()) telemetry.addData("Path", "point %d of %d   (%.1f in to go)",
                    progress, pathPts.size() - 1, remainingLength());
        }
        telemetry.update();
    }
}

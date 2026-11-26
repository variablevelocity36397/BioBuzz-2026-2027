package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.skeletonarmy.marrow.BulkCaching;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.pedropathing.api.Paths.line;

/*
  POLLEN INTAKER - HuskyLens + Marrow WEAVER + PEDRO PATHING
  ----------------------------------------------------------
  PEDRO does all the driving, using the team's TUNED setup in pedro/Constants.java
  (Pinpoint pod offsets, wheel names/directions, Foresight numbers).

    1. LOOK:  no pollen in view -> turn in place until the HuskyLens sees some.
    2. PLAN:  each pollen box -> a spot on the field. Weaver plans a path through them.
    3. GO:    SMOOTH_CURVE = false (default): Weaver's path is cut into a few straight legs.
              For each leg Pedro TURNS IN PLACE to face it, then drives a STRAIGHT LINE to it. No diagonals.
              SMOOTH_CURVE = true: Pedro follows Weaver's whole smooth curve in one go.
              The intake turns on when a ball is close. The path ends OVERSHOOT_IN past the last ball.
    4. DONE:  keep the intake on a moment, then go back to LOOK. Runs until STOP.

  Extra rules:
    - SPUN TOO LONG: looked MAX_LOOK_SPIN_DEG without finding pollen -> drive to the next SEARCH_SPOTS spot, look again.
    - OBSTACLES: Weaver plans every path AROUND the OBSTACLES zones, and balls inside them are ignored.
    - 2 MISSES: every ball spot gets MAX_TRIES tries. Still there after that -> ignore it and move to another spot.

  Camera mount numbers (CAMERA_HEIGHT_IN, CAMERA_PITCH_DOWN_DEG, ...) are in PollenCollector.java.
*/
@BulkCaching
@Autonomous(name = "Pollen Intaker (Weaver + Pedro)", group = "Pollen")
public class Pollenintaker extends LinearOpMode {

    // ---------- Start spot (field inches, like the team's autos) ----------
    private static final double START_X = 72, START_Y = 9, START_HEADING_DEG = 90;

    // ---------- How it drives ----------
    private static final boolean SMOOTH_CURVE = false;   // false = turn in place + straight lines, true = Weaver curve
    private static final double LOOK_TURN     = 0.3;     // turning power while looking for pollen
    private static final double TURN_DONE_DEG = 5;       // turned "enough" when this close
    private static final double LEG_TIMEOUT   = 3.0;     // seconds - safety for one turn or one leg

    // ---------- Path ----------
    private static final double LEG_MIN_IN    = 8;       // straight legs at least this long
    private static final double OVERSHOOT_IN  = 12;      // end this far PAST the last ball so it's surely in
    private static final double INTAKE_ON_IN  = 18;      // intake on when a ball is this close
    private static final double SETTLE_TIME   = 0.5;     // intake keeps running this long at the end
    private static final double ROBOT_LENGTH_IN = 18, ROBOT_WIDTH_IN = 18;
    private static final double INTAKE_HALF_WIDTH_IN = 4;   // HALF the real intake width (Marrow docs)

    // ---------- Spun too long / missed too often -> go somewhere else ----------
    private static final double MAX_LOOK_SPIN_DEG = 540;   // 1.5 turns without seeing pollen -> move
    private static final int    MAX_TRIES = 2;             // tries per ball before giving up on it
    private static final double SAME_BALL_IN = 8;          // a ball this close to an old spot = the SAME ball
    // Places to go look for pollen (field inches) - PLACEHOLDERS, change to good spots on your field
    private final Point[] SEARCH_SPOTS = {
            new Point(48, 48), new Point(96, 48), new Point(96, 96), new Point(48, 96),
    };

    // ---------- Obstacles: Weaver plans paths AROUND these (field inches) ----------
    // PLACEHOLDER - measure where the HIVE and other field structures really are. Add as many as you need:
    //   new CircleZone(new Point(x, y), radius)    or    new PolygonZone(new Point(x1,y1), new Point(x2,y2), ...)
    private final List<Zone> OBSTACLES = Arrays.<Zone>asList(
            new CircleZone(new Point(72, 72), 12)             // e.g. the HIVE in the middle
    );

    // ---------- Safety: don't hit the HIVE / walls hard ----------
    private static final double ROBOT_HALF_IN = 9;          // half the robot's size (18 in robot)
    private static final double SLOW_GAP_IN = 12;           // closer than this to a wall/obstacle -> go SLOW
    private static final double SLOW_SPEED_IN_PER_S = 15;   // speed limit when slow (inches per second)
    private static final double STOP_GAP_IN = 3;            // closer than this AND driving toward it -> STOP + turn away
    private static final double BACK_AWAY_IN = 8;           // then drive this far away from it

    // ---------- Camera ----------
    private static final int    POLLEN_ID = 1, SCREEN_W = 320;
    private static final int    MIN_BOX_PX = 10, MIN_Y = 60, CONFIRM_FRAMES = 2;
    private static final double MIN_ASPECT = 0.6, MAX_ASPECT = 1.6;
    private static final double MAX_BALL_DIST_IN = 48;      // ignore anything estimated further than this
    private final Zone FIELD = new PolygonZone(              // ignore "balls" off the field
            new Point(2, 2), new Point(142, 2), new Point(142, 142), new Point(2, 142));

    private Follower follower;
    private RobotNames.Intake intake;
    private HuskyLens husky;

    private double x, y, heading;                 // where the robot is (inches, radians, + = left)
    private List<Point> balls = new ArrayList<>();
    private boolean intakeOn = false;
    private int seen = 0, collected = 0, lastSide = 1;
    private String status = "Ready";
    private double spun = 0, lastHeading = 0;            // how far we've turned while looking (radians)
    private int nextSpot = 0;                            // which SEARCH_SPOTS to go to next
    private final List<Point> triedSpots = new ArrayList<>();   // every ball spot we went for
    private final List<Integer> tries = new ArrayList<>();      // ...and how many times
    private double gap = 99;                             // inches to the nearest wall/obstacle
    private boolean slowMode = false, tooClose = false, drivingPath = false;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);                 // team's tuned Pedro (Pinpoint + Mecanum + Foresight)
        Constants.foresightConfig.maxVelocityConstraint.set(Double.POSITIVE_INFINITY);   // normal speed to start
        follower.setPose(PoseFactory.degrees().of(START_X, START_Y, START_HEADING_DEG));
        intake = new RobotNames.Intake(hardwareMap);

        husky = RobotNames.need(hardwareMap, HuskyLens.class, RobotNames.HUSKY, "HuskyLens");
        boolean cameraOk = husky.knock();
        husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        Weaver.setConfig(new PathConfig()
                .intakeWidth(INTAKE_HALF_WIDTH_IN)
                .robotWidth(ROBOT_LENGTH_IN)      // Marrow's "width" = along the forward axis
                .robotHeight(ROBOT_WIDTH_IN));

        telemetry.addData("HuskyLens", cameraOk ? "ready" : "NOT FOUND - check wiring / name 'husky lens'");
        telemetry.addData("Intake", intake.describe());
        telemetry.addData("Driving", SMOOTH_CURVE ? "Pedro follows the Weaver curve" : "Pedro: turn in place + straight lines");
        telemetry.update();
        waitForStart();
        try {
            update();
            lastHeading = heading;
            while (opModeIsActive()) {
                update();
                spun += Math.abs(Math.atan2(Math.sin(heading - lastHeading), Math.cos(heading - lastHeading)));
                lastHeading = heading;

                List<Point> seenBalls = lookForBalls();
                boolean gaveUp = dropBallsTriedTooOften(seenBalls);

                if (gaveUp || spun > Math.toRadians(MAX_LOOK_SPIN_DEG)) {
                    moveToNextSearchSpot(gaveUp ? "Missed that ball " + MAX_TRIES + " times" : "Spun too long, nothing here");
                    continue;
                }
                if (seenBalls.isEmpty()) {
                    status = "Looking - turning in place (" + Math.round(Math.toDegrees(spun)) + " deg)";
                    follower.manual(0, 0, LOOK_TURN * lastSide);     // forward, left, turn (+ = left)
                    show();
                    continue;
                }
                follower.manual(0, 0, 0);
                go(seenBalls);
                spun = 0;
                lastHeading = heading;
            }

        } finally {
            // ALWAYS put Pedro's speed limit back - it's shared with the team's other autos
            Constants.foresightConfig.maxVelocityConstraint.set(Double.POSITIVE_INFINITY);
            follower.stop();
            intake.setPower(0);
        }
    }

    // ======================= GIVE UP / MOVE SOMEWHERE ELSE =======================

    /** Counts a try for every ball we go for. Removes balls already tried MAX_TRIES times. True if any were removed. */
    private boolean dropBallsTriedTooOften(List<Point> seenBalls) {
        boolean dropped = false;
        for (int i = seenBalls.size() - 1; i >= 0; i--) {
            int k = triedIndex(seenBalls.get(i));
            if (k >= 0 && tries.get(k) >= MAX_TRIES) { seenBalls.remove(i); dropped = true; }
        }
        return dropped && seenBalls.isEmpty();     // only "give up" when nothing else worth trying is in view
    }

    private int triedIndex(Point ball) {
        for (int k = 0; k < triedSpots.size(); k++) if (triedSpots.get(k).distanceTo(ball) < SAME_BALL_IN) return k;
        return -1;
    }

    private void countTry(Point ball) {
        int k = triedIndex(ball);
        if (k < 0) { triedSpots.add(ball); tries.add(1); }
        else tries.set(k, tries.get(k) + 1);
    }

    /** Drive (around obstacles) to the next search spot, then start looking again. */
    private void moveToNextSearchSpot(String why) {
        Point spot = SEARCH_SPOTS[nextSpot];
        nextSpot = (nextSpot + 1) % SEARCH_SPOTS.length;
        status = why + " - moving to (" + Math.round(spot.getX()) + ", " + Math.round(spot.getY()) + ")";
        follower.manual(0, 0, 0);
        List<Point> route;
        try {
            route = Weaver.builder().start(new Point(x, y, heading)).end(spot).obstacles(OBSTACLES).build()
                    .getPath().sample(20);
        } catch (RuntimeException e) {
            route = new ArrayList<>();
            route.add(spot);
        }
        for (Point leg : thinOut(route)) {
            if (!opModeIsActive() || tooClose) break;
            turnToFace(leg);
            driveStraightTo(leg);
        }
        if (tooClose) backAway();
        spun = 0;
        seen = 0;
        lastHeading = heading;
    }

    // ======================= PLAN + GO =======================

    private void go(List<Point> targets) {
        intakeOn = false;
        PathResult result = null;
        try {
            result = Weaver.builder().start(new Point(x, y, heading)).targets(targets).obstacles(OBSTACLES).build();
            balls = new ArrayList<>(result.getVisitedTargets());
            if (balls.isEmpty()) throw new IllegalStateException("every ball is blocked");
        } catch (RuntimeException e) {          // Weaver couldn't -> straight at the closest ball
            Point closest = targets.get(0);
            for (Point p : targets) if (distTo(p) < distTo(closest)) closest = p;
            balls = new ArrayList<>();
            balls.add(closest);
            result = null;
        }
        for (Point b : balls) countTry(b);   // if it's still there next time we look, that was a miss

        if (SMOOTH_CURVE && result != null) {
            status = "Pedro following the Weaver curve";
            followAndWait(PedroPathingConverter.toPath(result));
            Point end = balls.get(balls.size() - 1);              // then a short straight push past the last ball
            if (!tooClose) driveStraightTo(pastPoint(new Point(x, y), end));
        } else {
            List<Point> legs = toStraightLegs(result != null ? result.getPath().sample(20) : balls);
            for (int i = 0; i < legs.size() && opModeIsActive() && !tooClose; i++) {
                status = "Leg " + (i + 1) + " of " + legs.size();
                turnToFace(legs.get(i));
                driveStraightTo(legs.get(i));
            }
        }
        if (tooClose) backAway();

        status = "Finishing pickup";
        follower.manual(0, 0, 0);
        intake.setPower(1);
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < SETTLE_TIME) { update(); show(); }
        intake.setPower(0);
        intakeOn = false;
        collected += balls.size();
        balls = new ArrayList<>();
        seen = 0;
    }

    /** Straight legs to the balls, with the last one pushed OVERSHOOT_IN past the end. */
    private List<Point> toStraightLegs(List<Point> curve) {
        List<Point> legs = thinOut(curve);
        Point before = legs.size() > 1 ? legs.get(legs.size() - 2) : new Point(x, y);
        legs.set(legs.size() - 1, pastPoint(before, legs.get(legs.size() - 1)));
        return legs;
    }

    /** Keep only points at least LEG_MIN_IN apart (always the last one) -> a few long straight legs. */
    private List<Point> thinOut(List<Point> curve) {
        List<Point> legs = new ArrayList<>();
        Point prev = new Point(x, y);
        for (int i = 0; i < curve.size(); i++) {
            Point p = curve.get(i);
            if (p.distanceTo(prev) >= LEG_MIN_IN || i == curve.size() - 1) { legs.add(p); prev = p; }
        }
        return legs;
    }

    /** The point OVERSHOOT_IN past 'end', going in the direction from 'from' to 'end'. */
    private static Point pastPoint(Point from, Point end) {
        double h = Math.atan2(end.getY() - from.getY(), end.getX() - from.getX());
        return new Point(end.getX() + OVERSHOOT_IN * Math.cos(h), end.getY() + OVERSHOOT_IN * Math.sin(h));
    }

    /** TURN IN PLACE (Pedro holds this spot with the new heading) until facing p. */
    private void turnToFace(Point p) {
        double target = Math.atan2(p.getY() - y, p.getX() - x);
        follower.hold(new Pose(x, y, target));
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < LEG_TIMEOUT) {
            update();
            double err = Math.atan2(Math.sin(target - heading), Math.cos(target - heading));
            if (Math.abs(err) < Math.toRadians(TURN_DONE_DEG)) break;
            show();
        }
    }

    /** DRIVE A STRAIGHT LINE to p, facing along the line. */
    private void driveStraightTo(Point p) {
        followAndWait(line(new Pose(x, y, heading), new Pose(p.getX(), p.getY())).tangent());
    }

    private void followAndWait(Path path) {
        if (tooClose) return;
        follower.follow(path);
        drivingPath = true;
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && !tooClose && t.seconds() < LEG_TIMEOUT * 2) {
            update();
            show();
        }
        drivingPath = false;
        if (tooClose) follower.hold(new Pose(x, y, heading));   // STOP right here
    }

    // ======================= SAFETY: walls + obstacles =======================

    /** Inches between the robot's edge and the nearest wall or obstacle at (px, py). */
    private double gapAt(double px, double py) {
        double g = Math.min(Math.min(px, 144 - px), Math.min(py, 144 - py)) - ROBOT_HALF_IN;
        Point here = new Point(px, py);
        for (Zone z : OBSTACLES) g = Math.min(g, z.distanceTo(here) - ROBOT_HALF_IN);
        return g;
    }

    /** Checked every loop: SLOW down near walls/obstacles, and STOP if about to hit one. */
    private void checkSafety() {
        gap = gapAt(x, y);
        boolean slow = gap < SLOW_GAP_IN;
        if (slow != slowMode) {               // Pedro reads this speed limit while it drives
            slowMode = slow;
            Constants.foresightConfig.maxVelocityConstraint.set(slow ? SLOW_SPEED_IN_PER_S : Double.POSITIVE_INFINITY);
        }
        double gapAhead = gapAt(x + 3 * Math.cos(heading), y + 3 * Math.sin(heading));
        if (drivingPath && gap < STOP_GAP_IN && gapAhead < gap) tooClose = true;   // driving INTO it -> stop
    }

    /** Too close: turn in place to face AWAY from the wall/obstacle, then drive straight away a bit. */
    private void backAway() {
        tooClose = false;
        status = "TOO CLOSE - turning away";
        double awayX = gapAt(x + 1, y) - gapAt(x - 1, y);       // which way gets us more room
        double awayY = gapAt(x, y + 1) - gapAt(x, y - 1);
        double away = Math.atan2(awayY, awayX);
        Point target = new Point(x + BACK_AWAY_IN * Math.cos(away), y + BACK_AWAY_IN * Math.sin(away));
        turnToFace(target);
        driveStraightTo(target);
        tooClose = false;
    }

    // ======================= EVERY LOOP =======================

    /** Pedro update + where we are + safety + intake on once a ball is close (stays on for the rest of the path). */
    private void update() {
        follower.update();
        Pose p = follower.pose();
        x = p.x(); y = p.y(); heading = p.heading();
        checkSafety();
        if (!intakeOn) for (Point b : balls) if (distTo(b) < INTAKE_ON_IN) intakeOn = true;
        intake.setPower(intakeOn ? 1 : 0);
    }

    // ======================= CAMERA =======================

    /** Every pollen box in view, as a spot on the field (empty until it's seen CONFIRM_FRAMES in a row). */
    private List<Point> lookForBalls() {
        List<Point> out = new ArrayList<>();
        HuskyLens.Block[] blocks = husky.blocks();
        if (blocks != null) {
            for (HuskyLens.Block b : blocks) {
                if (b.id != POLLEN_ID || b.width < MIN_BOX_PX || b.height < MIN_BOX_PX || b.y < MIN_Y) continue;
                double aspect = (double) b.width / b.height;
                if (aspect < MIN_ASPECT || aspect > MAX_ASPECT) continue;           // not round
                Point spot = toField(b);
                if (spot == null || !FIELD.contains(spot)) continue;                // too far / off the field
                if (insideObstacle(spot)) continue;                                 // can't drive there
                out.add(spot);
                lastSide = b.x <= SCREEN_W / 2 ? 1 : -1;                            // look that way next time
            }
        }
        seen = out.isEmpty() ? 0 : seen + 1;
        if (seen < CONFIRM_FRAMES) out.clear();
        return out;
    }

    /** Camera box -> field spot, from the camera's height and tilt (same math as Pollen Collector). */
    private Point toField(HuskyLens.Block b) {
        double rightDeg = (b.x - SCREEN_W / 2.0) / SCREEN_W * PollenCollector.HFOV_DEG;
        double upDeg = (PollenCollector.SCREEN_H / 2.0 - b.y) / PollenCollector.SCREEN_H * PollenCollector.VFOV_DEG;
        double downDeg = PollenCollector.CAMERA_PITCH_DOWN_DEG - upDeg;
        if (downDeg < 2) return null;                                               // near the horizon
        double dist = (PollenCollector.CAMERA_HEIGHT_IN - PollenCollector.BALL_RADIUS_IN) / Math.tan(Math.toRadians(downDeg));
        if (dist > MAX_BALL_DIST_IN) return null;
        double fwd = PollenCollector.CAMERA_FORWARD_IN + dist;
        double left = -dist * Math.tan(Math.toRadians(rightDeg));
        return new Point(x + fwd * Math.cos(heading) - left * Math.sin(heading),
                         y + fwd * Math.sin(heading) + left * Math.cos(heading));
    }

    private double distTo(Point p) { return Math.hypot(p.getX() - x, p.getY() - y); }

    private boolean insideObstacle(Point p) {
        for (Zone z : OBSTACLES) if (z.contains(p)) return true;
        return false;
    }

    private void show() {
        telemetry.addData("Status", status);
        telemetry.addData("Collected", collected);
        telemetry.addData("Intake", intakeOn ? "ON" : "off");
        telemetry.addData("Robot at", "x %.1f  y %.1f  heading %.0f deg", x, y, Math.toDegrees(heading));
        telemetry.addData("Safety", "%.1f in to wall/obstacle%s", gap, tooClose ? "  TOO CLOSE" : slowMode ? "  (slow)" : "");
        telemetry.addData("Pedro busy", follower.isBusy());
        telemetry.update();
    }
}

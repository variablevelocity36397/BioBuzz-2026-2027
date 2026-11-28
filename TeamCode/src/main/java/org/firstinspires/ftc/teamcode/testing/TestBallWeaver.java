package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.vision.HuskyBallCamera;

import java.util.ArrayList;
import java.util.List;

/**
 * Sees 1-4 balls with the HuskyLens, Weaver builds one path through them, and the path is
 * regenerated every REPLAN_SEC if the balls moved. Only A starts a run; it stops when the path ends.
 * B or any stick = stop.
 */
@TeleOp(name = "Test Ball Weaver", group = "Test")
public class TestBallWeaver extends OpMode {
    // Red garden: SimpleRedAuto's garden pose (blue 130, 133.1, 0 rotated 180 deg about field center)
    public static double START_X = 14, START_Y = 10.9, START_HEADING_DEG = 180;
    // Hive keep-out rectangle (field inches): X/Y = its CENTER, W along x, H along y. Must be measured; the robot won't drive while any is NaN.
    public static double HIVE_X = 70.7, HIVE_Y = 70.7, HIVE_W = 50, HIVE_H = 46;
    public static double INTAKE_WIDTH = 10;  // in
    // Marrow: robotWidth is along the robot's forward axis (length), robotHeight is side to side
    public static double ROBOT_W = 21, ROBOT_H = 18, CLEARANCE = 4;
    public static double REPLAN_SEC = 0.5;
    public static double MOVE_TOL = 4;       // in, a ball this far from its planned spot counts as moved
    public static double REACHED_DIST = 4;   // in, planned ball counts as collected once the robot is this close

    private Follower follower;
    private final HuskyBallCamera camera = new HuskyBallCamera();
    private Intake intake;
    private final ElapsedTime replanTimer = new ElapsedTime();
    private List<Point> targets = new ArrayList<>(); // planned balls not yet reached, in visit order
    private boolean running;
    private String status = "Press A to scan and go";

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        follower.update();
        camera.init(hardwareMap);
        intake = new Intake(hardwareMap);
        Weaver.setConfig(new PathConfig().intakeWidth(INTAKE_WIDTH).robotWidth(ROBOT_W).robotHeight(ROBOT_H).clearance(CLEARANCE));
    }

    @Override
    public void init_loop() {
        follower.update();
        telemetryBalls(camera.ballsOnField(follower.pose()));
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();
        Point robot = new Point(pose.x(), pose.y());
        targets.removeIf(t -> t.distanceTo(robot) < REACHED_DIST);

        boolean sticks = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x) > 0.1;
        if (running && (gamepad1.bWasPressed() || sticks)) finish("Stopped by driver");

        List<Point> seen = camera.ballsOnField(pose);
        if (!running && gamepad1.aWasPressed()) {
            running = plan(seen);
            replanTimer.reset();
        } else if (running && replanTimer.seconds() >= REPLAN_SEC) {
            replanTimer.reset();
            if (moved(seen, pose)) {
                // Keep planned balls the camera can't check right now (out of view or under the intake)
                List<Point> merged = new ArrayList<>(seen);
                for (Point t : targets) if (!HuskyBallCamera.canSee(t, pose)) merged.add(t);
                if (!plan(merged)) finish(status);
            }
        }
        if (running && !follower.isBusy()) finish("Done");

        intake.set(running);
        intake.update();

        telemetry.addData("Status", status);
        telemetry.addData("Running", running);
        telemetry.addData("Planned balls left", targets.size());
        telemetry.addData("Pose (x, y, heading deg)", "%.1f, %.1f, %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetryBalls(seen);
        telemetry.update();
    }

    /** Generates and starts a Weaver path through the balls, always avoiding the hive. False if nothing to drive to. */
    private boolean plan(List<Point> balls) {
        if (Double.isNaN(HIVE_X + HIVE_Y + HIVE_W + HIVE_H)) { status = "HIVE zone not set: won't drive"; return false; }
        if (balls.isEmpty()) { status = "No balls seen"; return false; }
        PathResult result = Weaver.builder()
                .start(PedroPathingConverter.toPoint(follower.pose()))
                .targets(balls)
                .maxTargets(HuskyBallCamera.MAX_BALLS)
                .obstacles(new PolygonZone(new Point(HIVE_X, HIVE_Y), HIVE_W, HIVE_H))
                .generate();
        targets = new ArrayList<>(result.getVisitedTargets());
        if (targets.isEmpty()) { status = "All seen balls blocked by hive"; return false; }
        follower.follow(PedroPathingConverter.toPath(result));
        status = "Following path to " + targets.size() + " ball(s)";
        return true;
    }

    /** True if a seen ball isn't near any planned ball, or a planned ball in view is no longer seen. */
    private boolean moved(List<Point> seen, Pose pose) {
        for (Point s : seen) if (nearest(s, targets) > MOVE_TOL) return true;
        for (Point t : targets) if (HuskyBallCamera.canSee(t, pose) && nearest(t, seen) > MOVE_TOL) return true;
        return false;
    }

    private static double nearest(Point p, List<Point> others) {
        double best = Double.MAX_VALUE;
        for (Point o : others) best = Math.min(best, p.distanceTo(o));
        return best;
    }

    private void finish(String why) {
        follower.stop();
        running = false;
        targets.clear();
        status = why + " - press A to scan again";
    }

    private void telemetryBalls(List<Point> balls) {
        if (!camera.isConnected()) telemetry.addLine("!!! HUSKYLENS NOT RESPONDING (check I2C config name 'huskylens') !!!");
        if (Double.isNaN(HIVE_X + HIVE_Y + HIVE_W + HIVE_H)) telemetry.addLine("!!! Set HIVE_X/Y/W/H before driving !!!");
        telemetry.addData("Balls seen", balls.size());
        for (Point b : balls) telemetry.addData("  ball", "%.1f, %.1f", b.getX(), b.getY());
    }
}

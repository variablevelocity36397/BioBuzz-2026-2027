package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.skeletonarmy.marrow.zones.Point;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/** HuskyLens color recognition: turns seen balls into field points via ground-plane projection. */
public class HuskyBallCamera {
    // Measure all of these on the robot. Angles in degrees, distances in inches.
    public static double CAM_HEIGHT = 9;     // lens height above floor
    public static double CAM_PITCH = 30;     // lens tilt below horizontal
    public static double CAM_X = 4.5;        // lens forward of robot center, measured with intake extended
    public static double CAM_Y = 0;          // lens left of robot center
    public static double CAM_YAW = 0;        // lens yaw, + = pointed left of robot forward
    public static double HFOV = 60;          // horizontal field of view (published ~60 deg, unverified)
    public static double VFOV = 46.8;        // vertical, from HFOV at the 4:3 frame: 2*atan(tan(30)*3/4)
    public static double BALL_RADIUS = 1.5;  // pollen is ~3 in diameter
    public static int BALL_COLOR_ID = 1;     // HuskyLens learned color ID
    public static int MIN_BLOCK_PX = 6;      // smaller blocks are noise
    public static int MAX_BALLS = 4;

    private static final double FRAME_W = 320, FRAME_H = 240, FIELD = 144;

    private HuskyLens husky;
    private boolean connected;

    public void init(HardwareMap hardwareMap) {
        husky = hardwareMap.get(HuskyLens.class, "huskylens");
        connected = husky.knock();
        if (connected) husky.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public boolean isConnected() { return connected; }

    /** Up to MAX_BALLS seen balls in field coordinates, nearest first. Empty if the camera is down. */
    public List<Point> ballsOnField(Pose robot) {
        List<Point> balls = new ArrayList<>();
        if (!connected) return balls;
        for (HuskyLens.Block b : husky.blocks()) {
            if (b.id != BALL_COLOR_ID || b.width < MIN_BLOCK_PX) continue;
            Point p = toField(b.x, b.y, robot);
            if (p != null && p.getX() >= 0 && p.getX() <= FIELD && p.getY() >= 0 && p.getY() <= FIELD) balls.add(p);
        }
        Point r = new Point(robot.x(), robot.y());
        balls.sort(Comparator.comparingDouble(r::distanceTo));
        return balls.size() > MAX_BALLS ? new ArrayList<>(balls.subList(0, MAX_BALLS)) : balls;
    }

    /** Pixel (ball center) to field point, or null if the pixel is at/above the horizon. */
    public static Point toField(double px, double py, Pose robot) {
        double down = Math.toRadians(CAM_PITCH + (py - FRAME_H / 2) / FRAME_H * VFOV);
        if (down < Math.toRadians(1)) return null;
        double fwd = (CAM_HEIGHT - BALL_RADIUS) / Math.tan(down);
        double left = -fwd * Math.tan(Math.toRadians((px - FRAME_W / 2) / FRAME_W * HFOV));
        double yaw = Math.toRadians(CAM_YAW);
        double rx = CAM_X + fwd * Math.cos(yaw) - left * Math.sin(yaw);
        double ry = CAM_Y + fwd * Math.sin(yaw) + left * Math.cos(yaw);
        double h = robot.heading();
        return new Point(robot.x() + rx * Math.cos(h) - ry * Math.sin(h),
                         robot.y() + rx * Math.sin(h) + ry * Math.cos(h));
    }

    /** True if a ball at this field point would be inside the camera's view right now. */
    public static boolean canSee(Point p, Pose robot) {
        double dx = p.getX() - robot.x(), dy = p.getY() - robot.y(), h = robot.heading();
        double rx = dx * Math.cos(h) + dy * Math.sin(h) - CAM_X;
        double ry = -dx * Math.sin(h) + dy * Math.cos(h) - CAM_Y;
        double yaw = Math.toRadians(CAM_YAW);
        double fwd = rx * Math.cos(yaw) + ry * Math.sin(yaw);
        double left = -rx * Math.sin(yaw) + ry * Math.cos(yaw);
        if (fwd <= 0) return false;
        if (Math.abs(Math.toDegrees(Math.atan2(left, fwd))) > HFOV / 2) return false;
        double down = Math.toDegrees(Math.atan2(CAM_HEIGHT - BALL_RADIUS, fwd));
        return Math.abs(down - CAM_PITCH) <= VFOV / 2;
    }
}

package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsytems.TurretTrack;

/**
 * Turns a Limelight field pose into a Pedro robot pose. The Limelight must have its camera offset set to zero in its web
 * UI: the camera rides on the turret, so its pose is the camera's, and the turret angle is applied here instead.
 */
public final class VisionPose {
    // Limelight/FTC field frame (meters, origin at field center) -> Pedro (inches, origin at a corner).
    // Rotates by FIELD_ROTATION_DEG about the center, then shifts by FIELD_CENTER_IN. Default is Pedro's FTC conversion
    // (x = ftcY + 72, y = 72 - ftcX, heading - 90). Verify on the robot in Test Vision Pose before enabling.
    public static final double FIELD_ROTATION_DEG = -90;
    public static final double FIELD_CENTER_IN = 72;
    // Camera position in the turret frame, measured from the turret pivot (in): forward and left of the pivot.
    public static final double CAMERA_FORWARD_OF_PIVOT_IN = 0;
    public static final double CAMERA_LEFT_OF_PIVOT_IN = 0;
    private static final double METERS_TO_IN = 39.3700787;

    private VisionPose() {}

    /**
     * Robot pose (Pedro frame) from the camera's field pose.
     * robotHeadingRad = robot heading when the picture was taken (odometry; only used to place the pivot).
     * turretDeg = turret angle then (+ = left). The returned heading is the camera heading minus the turret angle, so
     * the caller can compare it with odometry as a sanity check.
     */
    public static Pose robotFromCamera(double camXMeters, double camYMeters, double camYawDeg,
                                       double turretDeg, double robotHeadingRad) {
        double r = Math.toRadians(FIELD_ROTATION_DEG);
        double fx = camXMeters * METERS_TO_IN, fy = camYMeters * METERS_TO_IN;
        double camX = FIELD_CENTER_IN + fx * Math.cos(r) - fy * Math.sin(r);
        double camY = FIELD_CENTER_IN + fx * Math.sin(r) + fy * Math.cos(r);
        double camHeading = Math.toRadians(camYawDeg) + r;

        // Camera offset from the robot's tracking center, in the robot frame (pivot, then camera around the pivot)
        double t = Math.toRadians(turretDeg);
        double ox = TurretTrack.TURRET_OFFSET_X
                + CAMERA_FORWARD_OF_PIVOT_IN * Math.cos(t) - CAMERA_LEFT_OF_PIVOT_IN * Math.sin(t);
        double oy = TurretTrack.TURRET_OFFSET_Y
                + CAMERA_FORWARD_OF_PIVOT_IN * Math.sin(t) + CAMERA_LEFT_OF_PIVOT_IN * Math.cos(t);
        double ch = Math.cos(robotHeadingRad), sh = Math.sin(robotHeadingRad);
        double x = camX - (ox * ch - oy * sh);
        double y = camY - (ox * sh + oy * ch);
        return new Pose(x, y, camHeading - Math.toRadians(turretDeg));
    }
}

package org.firstinspires.ftc.teamcode.Vision;

// Turns the Limelight's ty into a shooter velocity, farther away = lower ty = faster flywheel
public class ShooterSpeed {

    // {ty degrees, velocity ticks/s}, sorted from biggest ty (closest) to smallest (farthest)
    // PLACEHOLDERS: park at a few distances, find the velocity that scores, write down ty from telemetry
    private static final double[][] TABLE = {
            {15, 1800},
            {5, 2100},
            {-12, 1550},
    };

    // Straight line between the two closest table rows, clamped to the ends of the table
    public static double fromTy(double ty) {
        if (ty >= TABLE[0][0]) {
            return TABLE[0][1];
        }
        int last = TABLE.length - 1;
        if (ty <= TABLE[last][0]) {
            return TABLE[last][1];
        }

        for (int i = 0; i < last; i++) {
            double[] near = TABLE[i];
            double[] far = TABLE[i + 1];
            if (ty <= near[0] && ty >= far[0]) {
                double t = (near[0] - ty) / (near[0] - far[0]);  // 0 at near row, 1 at far row
                return near[1] + t * (far[1] - near[1]);
            }
        }
        return TABLE[last][1];  // never reached
    }
}

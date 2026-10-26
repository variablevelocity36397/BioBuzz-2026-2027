package org.firstinspires.ftc.teamcode.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Auto version of AutoAlign: Pedro owns the wheels in auto, so we give it a heading to hold instead of setting motor power
public class PedroAlign {

    private LimeLightCamera camera;
    private Follower follower;

    private static final double TOLERANCE = 1.0;                       // degrees of tx considered aligned
    private static final double HEADING_SETTLED = Math.toRadians(1.0); // Pedro is done turning when this close
    private static final double TIMEOUT = 1.5;                         // seconds before giving up
    private static final double LOST_TIMEOUT = 0.3;                    // seconds without the tag before giving up
    private static final int MAX_ATTEMPTS = 3;

    private double targetHeading;  // radians
    private int attempts;
    private boolean done = true;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime lostTimer = new ElapsedTime();

    public PedroAlign(LimeLightCamera camera, Follower follower) {
        this.camera = camera;
        this.follower = follower;
    }

    public void start() {
        timer.reset();
        lostTimer.reset();
        attempts = 0;
        done = false;
        targetHeading = follower.pose().heading();
        aimAtTag();
    }

    // Turn in place so the tag ends up in the crosshair
    private void aimAtTag() {
        double tx = camera.getTargetTx();
        if (Double.isNaN(tx)) {
            return;
        }
        Pose pose = follower.pose();
        // +tx = tag to the right = clockwise = negative heading in Pedro, verify on robot
        targetHeading = pose.heading() - Math.toRadians(tx);
        follower.hold(pose.withHeading(targetHeading));
        attempts++;
    }

    // stay where we are and stop aligning
    private void finish() {
        follower.hold(follower.pose());
        done = true;
    }

    // Call once per loop after camera.update(). Returns true when aligned or gave up.
    public boolean update() {
        if (done) {
            return true;
        }

        if (timer.seconds() > TIMEOUT) {
            finish();
            return true;
        }

        double tx = camera.getTargetTx();
        if (Double.isNaN(tx)) {
            if (lostTimer.seconds() > LOST_TIMEOUT) {
                finish();
                return true;
            }
            return false;
        }
        lostTimer.reset();

        // wait for Pedro to finish the turn before checking tx again
        if (Math.abs(Angle.normalizeSigned(targetHeading - follower.pose().heading())) > HEADING_SETTLED) {
            return false;
        }

        if (Math.abs(tx) < TOLERANCE || attempts >= MAX_ATTEMPTS) {
            finish();
            return true;
        }

        aimAtTag();
        return false;
    }

    public boolean isDone() {
        return done;
    }

    public void addDebugTelemetry(Telemetry telemetry) {
        telemetry.addData("Align attempts", attempts);
        telemetry.addData("Align time", "%.2f", timer.seconds());
        telemetry.addData("Align target heading", "%.1f", Math.toDegrees(targetHeading));
    }
}

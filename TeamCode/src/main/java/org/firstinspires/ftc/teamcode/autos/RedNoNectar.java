package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Red, no nectar, paths only, no mechanisms. Rewritten from the Turtle Tracer export; paths, poses and headings unchanged. */
@Autonomous(name = "RedNoNectar", group = "Autonomous")
public class RedNoNectar extends OpMode {

    private enum State { LINE1, LINE2, LINE3, DONE }

    private Follower follower;
    private State state;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(62.787, 133.805, 90.000);
    private final Pose line1Start = poseFactory.of(62.787, 133.805, 0.0);
    private final Pose line1End = poseFactory.of(59.035, 106.431, 0.0);
    private final Pose line2End = poseFactory.of(52.064, 125.752, 0.0);
    private final Pose line3End = poseFactory.of(14.913, 115.594, 0.0);

    // Entering a state starts its path once
    private void setState(State next) {
        state = next;
        switch (next) {
            case LINE1: follower.follow(line1()); break;
            case LINE2: follower.follow(line2()); break;
            case LINE3: follower.follow(line3()); break;
            case DONE:  break;
        }
    }

    private void updateStateMachine() {
        switch (state) {
            case LINE1: if (!follower.isBusy()) setState(State.LINE2); break;
            case LINE2: if (!follower.isBusy()) setState(State.LINE3); break;
            case LINE3: if (!follower.isBusy()) setState(State.DONE);  break;
            case DONE:  break;
        }
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
    }

    @Override
    public void start() {
        setState(State.LINE1);
    }

    @Override
    public void loop() {
        follower.update();
        updateStateMachine();

        telemetry.addData("state", state);
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }
    }

    public Path line1() {
        return Paths.line(line1Start, line1End).constant(Math.toRadians(90));
    }

    public Path line2() {
        return Paths.line(line1End, line2End).constant(Math.toRadians(90));
    }

    public Path line3() {
        return Paths.line(line2End, line3End).constant(Math.toRadians(90));
    }
}

package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/** SimpleBlueAuto's paths only, no mechanisms. */
@Autonomous(name = "SimpleBluePath", group = "Autonomous")
public class SimpleBluePath extends OpMode {

    private enum State { GARDEN, PARK, DONE }

    private Follower follower;
    private State state;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(85.7848, 133.1263, 0);
    private final Pose garden = poseFactory.of(130, 133.1, 0);
    private final Pose parkStart = poseFactory.of(130, 133.1, 0);
    private final Pose park = poseFactory.of(127.5047, 45.1531, 145);
    private final Pose parkControl1 = poseFactory.of(105.2935, 92.8837, 0);

    // Entering a state starts its path once
    private void setState(State next) {
        state = next;
        switch (next) {
            case GARDEN: follower.follow(garden()); break;
            case PARK:   follower.follow(park());   break;
            case DONE:   break;
        }
    }

    private void updateStateMachine() {
        switch (state) {
            case GARDEN: if (!follower.isBusy()) setState(State.PARK); break;
            case PARK:   if (!follower.isBusy()) setState(State.DONE); break;
            case DONE:   break;
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
        setState(State.GARDEN);
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

    public Path garden() {
        return Paths.line(start, garden).linear(start, garden);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}

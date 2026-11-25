package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Blue tip side, paths only, no mechanisms. */
@Autonomous(name = "BlueTipSideAuto", group = "Autonomous")
public class BlueTipsideAuto extends OpMode {

    private enum State { GARDEN, SHOOT, PREP_FOR_FLOWER, FLOWER, LEAVE_FLOWER, PARK, DONE }

    private Follower follower;
    private State state;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(85.7848, 133.1263, 0);
    private final Pose gardenPath = poseFactory.of(130, 133.1, 0);
    private final Pose shootPathStart = poseFactory.of(130, 133.1, 0);
    private final Pose shootPath = poseFactory.of(109.777, 12.185, 270);
    private final Pose shootPathControl1 = poseFactory.of(102.5265, 74.1728, 0);
    private final Pose prepForFlower = poseFactory.of(94.4971, 31.22, 270);
    private final Pose flowerPath = poseFactory.of(94.8948, 12.3532, 270);
    private final Pose leaveFlower = poseFactory.of(94.7311, 26.3229, 270);
    private final Pose parkPathStart = poseFactory.of(94.7311, 26.3229, 270);
    private final Pose parkPath = poseFactory.of(129.9957, 21.5837, 180);

    // Entering a state starts its path once
    private void setState(State next) {
        state = next;
        switch (next) {
            case GARDEN:          follower.follow(gardenPath());    break;
            case SHOOT:           follower.follow(shootPath());     break;
            case PREP_FOR_FLOWER: follower.follow(prepForFlower()); break;
            case FLOWER:          follower.follow(flowerPath());    break;
            case LEAVE_FLOWER:    follower.follow(leaveFlower());   break;
            case PARK:            follower.follow(parkPath());      break;
            case DONE:            break;
        }
    }

    private void updateStateMachine() {
        switch (state) {
            case GARDEN:          if (!follower.isBusy()) setState(State.SHOOT);           break;
            case SHOOT:           if (!follower.isBusy()) setState(State.PREP_FOR_FLOWER); break;
            case PREP_FOR_FLOWER: if (!follower.isBusy()) setState(State.FLOWER);          break;
            case FLOWER:          if (!follower.isBusy()) setState(State.LEAVE_FLOWER);    break;
            case LEAVE_FLOWER:    if (!follower.isBusy()) setState(State.PARK);            break;
            case PARK:            if (!follower.isBusy()) setState(State.DONE);            break;
            case DONE:            break;
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

    public Path gardenPath() {
        return Paths.line(start, gardenPath).linear(start, gardenPath);
    }

    public Path shootPath() {
        return Paths.curve(shootPathStart, shootPathControl1, shootPath).linear(shootPathStart, shootPath);
    }

    public Path prepForFlower() {
        return Paths.line(shootPath, prepForFlower).constant(prepForFlower);
    }

    public Path flowerPath() {
        return Paths.line(prepForFlower, flowerPath).constant(flowerPath);
    }

    public Path leaveFlower() {
        return Paths.line(flowerPath, leaveFlower).constant(leaveFlower);
    }

    public Path parkPath() {
        return Paths.line(parkPathStart, parkPath).linear(parkPathStart, parkPath);
    }
}
